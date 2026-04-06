#pragma once
#include <condition_variable>

#include <mc_control/mc_global_controller.h>

#include "ControlMode.h"
#include "URControlType.h"
#include "mc_rtde/tools/ToolInterface.h"

#include <mc_rtc/logging.h>
#include <mc_rtde/DriverBridgeRTDE.h>
#include <mc_rtde/DriverBridgeURModernDriver.h>
#include <mc_rtde/tools/ToolGripperRobotiq.h>

namespace mc_rtde
{

template<ControlMode cm>
struct URControlLoop
{
  URControlLoop(const std::string & name, const mc_control::Configuration & config, double cycle_s);

  void init(mc_control::MCGlobalController & controller);

  void updateSensors(mc_control::MCGlobalController & controller);

  void updateControl(mc_control::MCGlobalController & controller);

  void controlThread(mc_control::MCGlobalController & controller,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

  void setActiveRobot(mc_control::MCGlobalController & controller,
                      std::string & active_name,
                      std::mutex & startM,
                      std::condition_variable & startCV,
                      bool & start,
                      bool & running);

  void attachTool(const std::string tool_name, const mc_control::Configuration & tool_config);

  void toolThread(const std::string & tool_name,
                  std::mutex & startM,
                  std::condition_variable & startCV,
                  bool & start,
                  bool & running);

  std::string activeName()
  {
    return active_name_;
  }

  std::unordered_map<std::string, std::shared_ptr<ToolInterface>> tools()
  {
    return toolsInterfaces_;
  }

  void autoCalibrateTools()
  {
    for(auto & tool : toolsInterfaces_) tool.second->autoCalibrate();
  }

private:
  mc_control::Configuration config_;

  const std::string name_;
  std::string active_name_;
  std::string ip_;
  rbd::MultiBodyConfig command_;
  URSensorInfo state_;

  mc_rtc::Logger logger_;
  size_t sensor_id_ = 0;
  size_t control_id_ = 0;
  size_t prev_control_id_ = 0;
  double delay_ = 0;
  double cycle_s_ = 0;

  std::unique_ptr<DriverBridge> driverBridge_{nullptr};
  URControlType<cm> control_;

  std::unordered_map<std::string, std::shared_ptr<ToolInterface>> toolsInterfaces_ = {};
  std::vector<std::thread> toolsThreads_ = {};
  std::atomic<bool> tools_running_{false};

  // To protect against concurrent read/write between:
  // - the thread URControlLoop::controlThread
  // - the method URControlLoop::updateSensors called from the controller_run thread (before MCGlobalController::run)
  mutable std::mutex updateSensorsMutex_;
  mutable std::mutex updateControlMutex_;

  std::vector<double> sensorsBuffer_ = std::vector<double>(6, 0.0);
};

template<ControlMode cm>
using URControlLoopPtr = std::unique_ptr<URControlLoop<cm>>;

template<ControlMode cm>
URControlLoop<cm>::URControlLoop(const std::string & name, const mc_control::Configuration & config, double cycle_s)
: name_(name), config_(config), logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-rtde-" + name_), cycle_s_(cycle_s)
{
  active_name_ = name_;
  ip_ = std::string(config_("ip"));
  std::string driverName = config_("driver", std::string{"ur_rtde"});
  Driver driver = (driverName == "ur_rtde") ? Driver::ur_rtde : Driver::ur_modern_driver;
  if(driver == Driver::ur_rtde)
  {
    driverBridge_ = std::make_unique<DriverBridgeRTDE>(ip_);
  }
  else
  {
    driverBridge_ = std::make_unique<DriverBridgeURModernDriver>(ip_, cycle_s_);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::init(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("init {}", active_name_);

  // No need for thread synchronization here as the URControlLoop::controlThread is not yet running

  driverBridge_->sync(); // ensures that we got the first data
  logger_.start(controller.current_controller(), cycle_s_);
  logger_.addLogEntry("sensor_id", [this] { return sensor_id_; });
  logger_.addLogEntry("prev_control_id", [this]() { return prev_control_id_; });
  logger_.addLogEntry("control_id", [this]() { return control_id_; });
  logger_.addLogEntry("delay", [this]() { return delay_; });

  auto & robot = controller.controller().robots().robot(active_name_);
  auto & real = controller.controller().realRobots().robot(active_name_);
  state_.qIn_ = driverBridge_->getActualQ();
  state_.torqIn_ = driverBridge_->getJointTorques();
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexInMBC(i);
    robot.mbc().q[jIndex][0] = state_.qIn_[i];
    robot.mbc().jointTorque[jIndex][0] = state_.torqIn_[i];
  }

  updateSensors(controller);
  updateControl(controller);

  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

template<ControlMode cm>
void URControlLoop<cm>::updateSensors(mc_control::MCGlobalController & controller)
{
  auto & robot = controller.robots().robot(active_name_);
  using GC = mc_control::MCGlobalController;
  using set_sensor_t = void (GC::*)(const std::string &, const std::vector<double> &);
  auto updateSensor = [&controller, &robot, this](set_sensor_t set_sensor, const std::vector<double> & data)
  {
    assert(sensorsBuffer_.size() == 6);
    std::memcpy(sensorsBuffer_.data(), data.data(), 6 * sizeof(double));
    (controller.*set_sensor)(robot.name(), sensorsBuffer_);
  };
  {
    // protects against conurrent write from the URControlLoop::updatecontrol thread
    std::lock_guard<std::mutex> lock(updateSensorsMutex_);
    updateSensor(&GC::setEncoderValues, state_.qIn_);
    updateSensor(&GC::setEncoderVelocities, state_.dqIn_);
    updateSensor(&GC::setJointTorques, state_.torqIn_);
  }

  std::vector<double> tools_state;
  for(auto & tool : toolsInterfaces_)
  {
    std::vector<double> s = tool.second->getState();
    tools_state.insert(tools_state.end(), s.begin(), s.end());
  }

  size_t tool_state_idx = 0;
  for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
  {
    if(i < 6) continue;
    size_t jIndex = robot.jointIndexInMBC(i);
    const auto & j = robot.mb().joint(jIndex);

    if(j.dof() == 1 && !j.isMimic())
    {
      if(tool_state_idx < tools_state.size())
      {
        std::lock_guard<std::mutex> lock(updateSensorsMutex_);
        robot.mbc().q[jIndex][0] = tools_state[tool_state_idx];
        tool_state_idx++;
      }
      else
      {
        mc_rtc::log::warning("[mc_rtde] tool_state_idx {}", tool_state_idx);
        mc_rtc::log::error("[mc_rtde] Total number of tool states: {}", tools_state.size());
        mc_rtc::log::error_and_throw("[mc_rtde] Found many controllable joint. Did you add all tools to yaml?");
      }
    }
  }
}

template<ControlMode cm>
void URControlLoop<cm>::updateControl(mc_control::MCGlobalController & controller)
{
  std::vector<double> tools_command;
  {
    std::lock_guard<std::mutex> lock(updateControlMutex_);
    // In the same thread as MCGlobalController::run, thus we don't need synchronization here
    auto & robot = controller.robots().robot(active_name_);
    command_ = robot.mbc();

    const auto & rjo = robot.refJointOrder();
    for(size_t i = 6; i < rjo.size(); ++i)
    {
      auto jIndex = robot.jointIndexInMBC(i);
      if(!command_.q[jIndex].empty())
      {
        tools_command.push_back(command_.q[jIndex][0]);
      }
    }
  }

  {
    size_t tools_state_idx = 0;
    for(auto & tool : toolsInterfaces_)
    {
      int dof = tool.second->getDOF();
      std::vector<double> tool_command(tools_command.begin() + tools_state_idx,
                                       tools_command.begin() + tools_state_idx + dof);
      toolsInterfaces_[tool.first]->setCommand(tool_command);
      tools_state_idx += dof;
    }
  }

  control_id_++;
}

template<ControlMode cm>
void URControlLoop<cm>::controlThread(mc_control::MCGlobalController & controller,
                                      std::mutex & startM,
                                      std::condition_variable & startCV,
                                      bool & start,
                                      bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  while(running)
  {
    driverBridge_->sync();
    {
      std::lock_guard<std::mutex> lock(updateSensorsMutex_);
      state_.qIn_ = driverBridge_->getActualQ();
      state_.torqIn_ = driverBridge_->getJointTorques();
      driverBridge_->setDataRead();
    }
    auto command = rbd::MultiBodyConfig{};
    {
      std::lock_guard<std::mutex> lock(updateControlMutex_);
      command = command_;
    }
    using namespace std::chrono;
    auto time_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    control_.control(*driverBridge_, controller.robots().robot(active_name_), command);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::setActiveRobot(mc_control::MCGlobalController & controller,
                                       std::string & active_name,
                                       std::mutex & startMutex,
                                       std::condition_variable & startCV,
                                       bool & startControl,
                                       bool & running)
{
  std::string previous_name = active_name_;
  active_name_ = active_name;

  // Clear previous tools' threads, interfaces, and gui
  tools_running_ = false;
  for(auto & thread : toolsThreads_)
  {
    if(thread.joinable())
    {
      thread.join();
    }
  }
  toolsThreads_.clear();
  toolsInterfaces_.clear();
  controller.controller().gui()->removeElement({"RTDE"}, fmt::format("Auto calibrate all tools of {}", active_name_));
  for(const auto & tool : toolsInterfaces_)
  {
    controller.controller().gui()->removeElement({"RTDE"}, fmt::format("Auto calibrate {}", tool.first));
  }

  // Set up new robot and its tools
  if(active_name_ == name_)
  {
    mc_rtc::log::info("Using default configuration {}", active_name_);
  }
  else if(config_.has(active_name_))
  {
    const mc_control::Configuration active_config = config_(active_name);
    for(const std::string & tool_name : active_config.keys())
    {
      const mc_control::Configuration tool_config = active_config(tool_name);
      if(!tool_config.has("type"))
      {
        mc_rtc::log::error_and_throw("Tool configuration must contain 'type'");
      }
      attachTool(tool_name, tool_config);
    }
  }
  else
  {
    mc_rtc::log::warning("[mc_rtde] Robot variation {} configuration is not available", active_name_);
    active_name_ = previous_name;
    mc_rtc::log::info("[mc_rtde] Keep using robot {}", active_name_);
  };

  // Sync new robot's states
  std::vector<double> tools_state;
  for(auto & tool : toolsInterfaces_)
  {
    std::vector<double> s = tool.second->getState();
    tools_state.insert(tools_state.end(), s.begin(), s.end());
  }
  auto & robot = controller.controller().robots().robot(active_name_);
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexInMBC(i);
    if(i < state_.qIn_.size())
    {
      robot.mbc().q[jIndex][0] = state_.qIn_[i];
      robot.mbc().jointTorque[jIndex][0] = state_.torqIn_[i];
    }
    else
    {
      // TOFIX: the rest of the joint in q need to be initialized as well even if they are mimic
      robot.mbc().q[jIndex][0] = tools_state[i - 6];
    }
  }

  updateSensors(controller);
  updateControl(controller);

  // Set up tools' threads and gui
  if(!toolsInterfaces_.empty())
  {
    controller.controller().gui()->addElement(
        {"RTDE"}, mc_rtc::gui::Button(fmt::format("Auto calibrate all tools of {}", active_name_),
                                      [&]() { autoCalibrateTools(); }));
  }
  tools_running_ = true;
  for(const auto & tool : toolsInterfaces_)
  {
    controller.controller().gui()->addElement(
        {"RTDE"},
        mc_rtc::gui::Button(fmt::format("Auto calibrate {}", tool.first), [&tool]() { tool.second->autoCalibrate(); }));

    std::string tool_name = tool.first;
    toolsThreads_.emplace_back([this, tool_name, &startMutex, &startCV, &startControl, &running]()
                               { this->toolThread(tool_name, startMutex, startCV, startControl, running); });
  }
}

template<ControlMode cm>
void URControlLoop<cm>::attachTool(const std::string tool_name, const mc_control::Configuration & tool_config)
{
  if(tool_config("type") == "robotiq")
  {
    int port = tool_config("port", 63352);
    std::string ip = tool_config("ip", std::string(ip_));
    if(!toolsInterfaces_.count(tool_name))
    {
      mc_rtc::log::info("[mc_rtde] Connecting tool {}", tool_name);
      auto tool = std::make_shared<mc_rtde::ToolGripperRobotiq>(ip, port);
      tool->connect();
      toolsInterfaces_.try_emplace(tool_name, std::move(tool));
    }
    else
      mc_rtc::log::error_and_throw("[mc_rtde] Tool {} is already attached to the robot", tool_name);
  }
  else
  {
    mc_rtc::log::warning("[mc_rtde] Tool type : {} is not supported", tool_config("type"));
  }
}

template<ControlMode cm>
void URControlLoop<cm>::toolThread(const std::string & tool_name,
                                   std::mutex & startM,
                                   std::condition_variable & startCV,
                                   bool & start,
                                   bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }

  while(running && tools_running_)
  {
    {
      std::vector<double> tool_state = toolsInterfaces_[tool_name]->getPosition();
      toolsInterfaces_[tool_name]->setState(tool_state);
    }

    using namespace std::chrono;
    auto time_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    {
      toolsInterfaces_[tool_name]->control();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

} // namespace mc_rtde
