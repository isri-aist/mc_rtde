// SpaceVecAlg
#include <SpaceVecAlg/Conversions.h>

#include <mc_rbdyn/rpy_utils.h>

// mc_rtde
#include "MCControlRtde.h"

namespace mc_rtde
{

MCControlRtde::MCControlRtde(mc_control::MCGlobalController & controller, const std::string & host)
: globalController_(controller), host_(host), robot_(nullptr),
  logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc_rtde-" + controller.robot().name()), delay_(0.0)
{
  // Get parameter values from the configuration file
  URConfigParameter config_param;
  auto rtdeConfig = controller.configuration().config("RTDE");
  if(!rtdeConfig.has("JointSpeed"))
  {
    mc_rtc::log::warning("[mc_rtde] 'JointSpeed' config entry missing. Using default value: {}",
                         config_param.joint_speed_);
  }
  rtdeConfig("JointSpeed", config_param.joint_speed_);

  if(!rtdeConfig.has("JointAcceleration"))
  {
    mc_rtc::log::warning("[mc_rtde] 'JointAcceleration' config entry missing. Using default value: {}",
                         config_param.joint_acceleration_);
  }
  rtdeConfig("JointAcceleration", config_param.joint_acceleration_);

  auto robot_name = controller.robot().name();
  if(!rtdeConfig.has(robot_name))
  {
    mc_rtc::log::error(
        "[mc_rtde] A name that matches the controller robot name is not defined in the configuration file");
    return;
  }

  /* Get communication information */
  mc_rtc::Configuration config_robot = rtdeConfig(robot_name);
  if(!config_robot.has("IP"))
  {
    mc_rtc::log::warning("[mc_rtde] 'IP' config entry missing. Using default value: {}", config_param.targetIP_);
  }
  config_robot("IP", config_param.targetIP_);

  /* Connect to robot (real or simulation) */
  if(host != "simulation")
  {
    /* Try to connect via UDP to the robot */
    mc_rtc::log::info("[mc_rtde] Connecting to {} robot on address {}", robot_name, config_param.targetIP_);

    robot_ = new URControl(config_param);

    /* Get the sensor value once */
    if(!robot_->getState(state_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[mc_rtde] Unable to get data from robot");
    }
  }
  else
  {
    mc_rtc::log::info("[mc_rtde] Running simulation only. No connection to real robot");
    robot_ = new URControl(host, config_param);

    /* Set start state values */
    auto & robot = controller.controller().robots().robot(robot_name);
    robot_->setStartState(robot.stance(), state_);
  }

  /* Setup log entries */
  logger_.start(controller.current_controller(), controller.timestep());
  addLogEntryRobotInfo();
  logger_.addLogEntry("delay", [this]() { return delay_; });

  /* Run QP (every timestep ms) and send result joint commands to the robot */
  controller.init(state_.qIn_);
  controller.running = true;
  controller.controller().gui()->addElement(
      {"RTDE"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

  mc_rtc::log::info("[mc_rtde] interface initialized");
}

/* Destructor */
MCControlRtde::~MCControlRtde()
{
  delete robot_;
}

void MCControlRtde::robotControl()
{
  while(globalController_.running)
  {
    auto start_t_ = std::chrono::high_resolution_clock::now();
    auto robot_name = globalController_.robot().name();

    if(host_ != "simulation")
    {
      /* Store the data received by "recvData()" in "state" */
      robot_->getState(state_);
    }

    /* Send sensor readings to mc_rtc controller */
    globalController_.setEncoderValues(robot_name, state_.qIn_);
    globalController_.setEncoderVelocities(robot_name, state_.dqIn_);
    globalController_.setJointTorques(robot_name, state_.torqIn_);

    // Run the controller
    if(globalController_.run())
    {
      /* Update control value from the data in a robot */
      auto & robot = globalController_.controller().robots().robot(robot_name);
      const auto & rjo = robot.refJointOrder();
      for(size_t i = 0; i < UR_JOINT_COUNT; ++i)
      {
        auto jIndex = robot.jointIndexByName(rjo[i]);
        cmdData_.qOut_[i] = robot.mbc().q[jIndex][0];
        cmdData_.dqOut_[i] = robot.mbc().alpha[jIndex][0];
        cmdData_.torqOut_[i] = robot.mbc().jointTorque[jIndex][0];
      }

      if(host_ != "simulation")
      {
        /* Send command data */
        robot_->sendCmdData(cmdData_);
      }
      else
      {
        /* Loop back the value of "cmdData_" to "state_" */
        robot_->loopbackState(cmdData_, state_);
      }
    }

    /* Wait until next controller run */
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_t_).count();
    if(elapsed > globalController_.timestep() * 1000)
    {
      mc_rtc::log::warning("[mc_rtde] Loop time {} exeeded timestep of {} ms", elapsed,
                           globalController_.timestep() * 1000);
    }
    else
    {
      std::this_thread::sleep_for(
          std::chrono::microseconds(static_cast<unsigned int>((globalController_.timestep() * 1000 - elapsed)) * 1000));
    }

    /* Print controller data to the log */
    logger_.log();
    delay_ = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_t_).count();
  }

  robot_->controlFinished();
}

void MCControlRtde::addLogEntryRobotInfo()
{
  /* Sensors */
  /* Position(Angle) values */
  logger_.addLogEntry("measured_joint_position", [this]() -> const std::vector<double> & { return state_.qIn_; });
  /* Velocity values */
  logger_.addLogEntry("measured_joint_velocity", [this]() -> const std::vector<double> & { return state_.dqIn_; });
  /* Torque values */
  logger_.addLogEntry("measured_joint_torque", [this]() -> const std::vector<double> & { return state_.torqIn_; });

  /* Command data to send to the robot */
  /* Position(Angle) values */
  logger_.addLogEntry("sent_joint_position", [this]() -> const std::vector<double> & { return cmdData_.qOut_; });
  /* Velocity values */
  logger_.addLogEntry("sent_joint_velocity", [this]() -> const std::vector<double> & { return cmdData_.dqOut_; });
  /* Torque values */
  logger_.addLogEntry("sent_joint_torque", [this]() -> const std::vector<double> & { return cmdData_.torqOut_; });
}

} // namespace mc_rtde
