
#include <mc_rtc/logging.h>
#include "URControl.h"

namespace mc_rtde
{

/**
 * @brief Interface constructor and destructor
 * UDP connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
URControl::URControl(const URConfigParameter & config_param)
: ur_rtde_control_(nullptr), ur_rtde_receive_(nullptr), cm_(config_param.cm_), host_(""),
  joint_speed_(config_param.joint_speed_), joint_acceleration_(config_param.joint_acceleration_)
{
  try
  {
    ur_rtde_control_ = new ur_rtde::RTDEControlInterface(config_param.targetIP_);
    ur_rtde_receive_ = new ur_rtde::RTDEReceiveInterface(config_param.targetIP_);
  }
  catch(const std::exception& e)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_rtde] Could not connect to ur robot: {}", e.what());
  }
 }

/**
 * @brief Interface constructor and destructor
 * Running simulation only. No connection to real robot.
 *
 * @param host "simulation" only
 * @param config_param Configuration file parameters
 */
URControl::URControl(const std::string & host, const URConfigParameter & config_param)
: ur_rtde_control_(nullptr), ur_rtde_receive_(nullptr), cm_(config_param.cm_), host_(host),
  joint_speed_(config_param.joint_speed_), joint_acceleration_(config_param.joint_acceleration_)
{}

/**
 * @brief Save the data received from the robot
 * 
 * @param state Current sensor values information
 * @return true Success
 * @return false Could not receive
 */
bool URControl::getState(URSensorInfo & state)
{
  try
  {
    /* Save the data received from the robot in "state" */
    state.qIn_ = ur_rtde_receive_->getActualQ();
    state.dqIn_ = ur_rtde_receive_->getActualQd();
    //state.torqIn_ = ur_rtde_receive_->;
    if(state.qIn_.empty() || state.dqIn_.empty())
    {
      mc_rtc::log::error("[mc_rtde] Data could not be received");
      return false;
    }
  }
  catch(const std::exception& e)
  {
    mc_rtc::log::error("[mc_rtde] Data could not be received due to an error in the ur_rtde library: {}", e.what());
    return false;
  }
  return true;
}

/**
 * @brief Set start state values for simulation
 * 
 * @param stance Value defined by RobotModule
 * @param state Current sensor values information
 */
void URControl::setStartState(const std::map<std::string, std::vector<double>> & stance,
                                   URSensorInfo & state)
{
  /* Start stance */
  for(int i = 0; i < UR5E_JOINT_COUNT; ++i)
  {
    state.qIn_[i] = 0.0;
    state.dqIn_[i] = 0.0;
    state.torqIn_[i] = 0.0;
  }

  /* Set position(Angle) values */
  try
  {
    state.qIn_[0] = stance.at("shoulder_pan_joint")[0];
    state.qIn_[1] = stance.at("shoulder_lift_joint")[0];
    state.qIn_[2] = stance.at("elbow_joint")[0];
    state.qIn_[3] = stance.at("wrist_1_joint")[0];
    state.qIn_[4] = stance.at("wrist_2_joint")[0];
    state.qIn_[5] = stance.at("wrist_3_joint")[0];
  }
  catch(const std::exception& e)
  {
    mc_rtc::log::error("[mc_rtde] Failed to get the value defined by RobotModule: {}", e.what());
  }

  /* copy to private member state */
  stateIn_ = state;
};

/**
 * @brief Loop back the value of "data" to "state"
 * 
 * @param data Command data for sending to UR5e robot
 * @param state Current sensor values information
 */
void URControl::loopbackState(const URCommandData & data, URSensorInfo & state)
{
  /*  Set current sensor values */
  for(int i = 0; i < UR5E_JOINT_COUNT; ++i)
  {
    state.qIn_[i] = data.qOut_[i];
    state.dqIn_[i] = data.dqOut_[i];
    state.torqIn_[i] = data.torqOut_[i];
  }
}

/**
 * @brief Send control commands to the robot
 * 
 * @param sendData Command data for sending to UR5e robot
 */
void URControl::sendCmdData(URCommandData & sendData)
{
  const double speed_acceleration = 0.5;
  const bool asynchronous = true;
  const std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  const std::vector<int> selection_vector = {1, 1, 1, 1, 1, 1};
  const int force_type = 2;
  const std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};
  bool ret;

  if(host_ != "simulation")
  {
    try
    {
      /* Send command data */
      ret = false;
      switch(cm_)
      {
        case mc_rtde::ControlMode::Position:
          ret = ur_rtde_control_->moveJ(sendData.qOut_, joint_speed_, joint_acceleration_, asynchronous);
          break;
        case mc_rtde::ControlMode::Velocity:
          ret = ur_rtde_control_->speedJ(sendData.dqOut_, speed_acceleration);
          break;
        case mc_rtde::ControlMode::Torque:
          ret = ur_rtde_control_->forceMode(task_frame, selection_vector, sendData.torqOut_, force_type, limits);
          break;
      }
      if(!ret)
      {
        mc_rtc::log::error("[mc_rtde] Could not send command to UR5e robot");
      }
    }
    catch(const std::exception& e)
    {
      mc_rtc::log::error("[mc_rtde] The command could not be sent due to an error in the ur_rtde library: {}", e.what());
    }
  }
}

/**
 * @brief The control of the robot is finished
 */
void URControl::controlFinished()
{
  if(host_ != "simulation")
  {
    ur_rtde_control_->stopScript();
  }
}

} // namespace mc_rtde
