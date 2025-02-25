#pragma once

#include <condition_variable>

#include <mc_control/mc_global_controller.h>

#include "ControlMode.h"
#include "URControlType.h"
#include <mc_rtde/DriverBridgeRTDE.h>
#include <mc_rtde/DriverBridgeURModernDriver.h>

namespace mc_rtde
{
const std::vector<std::string> ROBOT_NAMES = {"ur5e", "ur10"};
const std::string CONFIGURATION_FILE = "/usr/local/etc/mc_rtde/mc_rtc_ur.yaml";

template<ControlMode cm>
struct URControlLoop
{
  URControlLoop(Driver driver, const std::string & name, const std::string & ip);

  void init(mc_control::MCGlobalController & controller);

  void updateSensors(mc_control::MCGlobalController & controller);

  void updateControl(mc_control::MCGlobalController & controller);

  void controlThread(mc_control::MCGlobalController & controller,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

private:
  std::string name_;
  mc_rtc::Logger logger_;
  size_t sensor_id_ = 0;
  rbd::MultiBodyConfig command_;
  size_t control_id_ = 0;
  size_t prev_control_id_ = 0;
  double delay_ = 0;

  URSensorInfo state_;

  std::unique_ptr<DriverBridge> driverBridge_{nullptr};
  URControlType<cm> control_;

  mutable std::mutex updateSensorsMutex_;
  mutable std::mutex updateControlMutex_;

  std::vector<double> sensorsBuffer_ = std::vector<double>(6, 0.0);
};

template<ControlMode cm>
using URControlLoopPtr = std::unique_ptr<URControlLoop<cm>>;

template<ControlMode cm>
URControlLoop<cm>::URControlLoop(Driver driver, const std::string & name, const std::string & ip)
: name_(name), logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-rtde-" + name_)
{
  if(driver == Driver::ur_rtde)
  {
    driverBridge_ = std::make_unique<DriverBridgeRTDE>(ip);
  }
  else
  {
    driverBridge_ = std::make_unique<DriverBridgeURModernDriver>(ip);
  }
}

template<ControlMode cm>
void URControlLoop<cm>::init(mc_control::MCGlobalController & controller)
{
  driverBridge_->sync();
  logger_.start(controller.current_controller(), 0.002);
  logger_.addLogEntry("sensor_id", [this] { return sensor_id_; });
  logger_.addLogEntry("prev_control_id", [this]() { return prev_control_id_; });
  logger_.addLogEntry("control_id", [this]() { return control_id_; });
  logger_.addLogEntry("delay", [this]() { return delay_; });

  auto & robot = controller.controller().robots().robot(name_);
  auto & real = controller.controller().realRobots().robot(name_);
  state_.qIn_ = driverBridge_->getActualQ();
  state_.torqIn_ = driverBridge_->getJointTorques();
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexInMBC(i);
    robot.mbc().q[jIndex][0] = state_.qIn_[i];
    robot.mbc().jointTorque[jIndex][0] = state_.torqIn_[i];
    std::cout << "Initial encoder state: " << state_.qIn_[i] << std::endl;
  }

  updateSensors(controller);
  updateControl(controller);

  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

template<ControlMode cm>
void URControlLoop<cm>::updateSensors(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(updateSensorsMutex_);
  auto & robot = controller.robots().robot(name_);
  using GC = mc_control::MCGlobalController;
  using set_sensor_t = void (GC::*)(const std::string &, const std::vector<double> &);
  auto updateSensor = [&controller, &robot, this](set_sensor_t set_sensor, const std::vector<double> & data)
  {
    assert(sensorsBuffer_.size() == 6);
    std::memcpy(sensorsBuffer_.data(), data.data(), 6 * sizeof(double));
    (controller.*set_sensor)(robot.name(), sensorsBuffer_);
  };

  updateSensor(&GC::setEncoderValues, state_.qIn_);
  updateSensor(&GC::setEncoderVelocities, state_.dqIn_);
  updateSensor(&GC::setJointTorques, state_.torqIn_);
}

template<ControlMode cm>
void URControlLoop<cm>::updateControl(mc_control::MCGlobalController & controller)
{
  std::unique_lock<std::mutex> lock(updateControlMutex_);
  auto & robot = controller.robots().robot(name_);
  command_ = robot.mbc();

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
    std::cout << "im here" << std::endl;
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
    std::cout << "i passed the wait" << std::endl;
  }

  int runi = 0;
  while(running)
  {
    std::cout << "run " << runi << std::endl;
    driverBridge_->sync();
    state_.qIn_ = driverBridge_->getActualQ();
    state_.torqIn_ = driverBridge_->getJointTorques();
    control_.control(*driverBridge_, controller.robots().robot(name_), command_);
    std::cout << "end run " << runi++ << std::endl;
  }
}

} // namespace mc_rtde
