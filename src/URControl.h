#pragma once

#include <Eigen/Core>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "ControlMode.h"

namespace mc_rtde
{
const std::vector<std::string> ROBOT_NAMES = {"ur5e", "ur10"};
const std::string CONFIGURATION_FILE = "/usr/local/etc/mc_rtde/mc_rtc_ur.yaml";
constexpr int UR_JOINT_COUNT = 6;

/**
 * @brief Configuration file parameters for mc_rtde
 */
struct URConfigParameter
{
  URConfigParameter() : cm_(ControlMode::Position), joint_speed_(1.05), joint_acceleration_(1.4), targetIP_("localhost")
  {
  }
  /* Communication information with a real robot */
  /* ControlMode (position, velocity or torque) */
  ControlMode cm_;
  /* joint speed [rad/s] (Valid only when ControlMode is Position) */
  double joint_speed_;
  /* joint acceleration [rad/s^2] (Valid only when ControlMode is Position) */
  double joint_acceleration_;
  /* Connection target IP */
  std::string targetIP_;
};

/**
 * @brief Current sensor values information of UR5e robot
 */
struct URSensorInfo
{
  URSensorInfo()
  {
    qIn_.resize(UR_JOINT_COUNT);
    dqIn_.resize(UR_JOINT_COUNT);
    torqIn_.resize(UR_JOINT_COUNT);
  }
  /* Position(Angle) values */
  std::vector<double> qIn_;
  /* Velocity values */
  std::vector<double> dqIn_;
  /* Torque values */
  std::vector<double> torqIn_;
};

/**
 * @brief Command data for sending to UR5e robot
 */
struct URCommandData
{
  URCommandData()
  {
    qOut_.resize(UR_JOINT_COUNT);
    dqOut_.resize(UR_JOINT_COUNT);
    torqOut_.resize(UR_JOINT_COUNT);
    for(int i = 0; i < UR_JOINT_COUNT; ++i)
    {
      qOut_[i] = 0.0;
      dqOut_[i] = 0.0;
      torqOut_[i] = 0.0;
    }
  }
  /* Position(Angle) values */
  std::vector<double> qOut_;
  /* Velocity values */
  std::vector<double> dqOut_;
  /* Torque values */
  std::vector<double> torqOut_;
};

/**
 * @brief mc_rtc control interface for UR5e robot
 */
class URControl
{
  /* Communication information with a real robot */
  ur_rtde::RTDEControlInterface * ur_rtde_control_;
  ur_rtde::RTDEReceiveInterface * ur_rtde_receive_;

  /* ControlMode (position, velocity or torque) */
  ControlMode cm_;
  /* joint speed [rad/s] (Valid only when ControlMode is Position) */
  double joint_speed_;
  /* joint acceleration [rad/s^2] (Valid only when ControlMode is Position) */
  double joint_acceleration_;
  /* Connection host, robot name or "simulation" */
  std::string host_;
  /* Current sensor values information */
  URSensorInfo stateIn_;

public:
  /**
   * @brief Interface constructor and destructor
   * ur_rtde connection with robot using specified parameters.
   *
   * @param config_param Configuration file parameters
   */
  URControl(const URConfigParameter & config_param);

  /**
   * @brief Interface constructor and destructor
   * Running simulation only. No connection to real robot.
   *
   * @param host "simulation" only
   * @param config_param Configuration file parameters
   */
  URControl(const std::string & host, const URConfigParameter & config_param);

  ~URControl()
  {
    delete ur_rtde_control_;
    delete ur_rtde_receive_;
  };

  /**
   * @brief Save the data received from the robot
   *
   * @param state Current sensor values information
   * @return true Success
   * @return false Could not receive
   */
  bool getState(URSensorInfo & state);

  /**
   * @brief Set the start state values for simulation
   *
   * @param stance Value defined by RobotModule
   * @param state Current sensor values information
   */
  void setStartState(const std::map<std::string, std::vector<double>> & stance, URSensorInfo & state);

  /**
   * @brief Loop back the value of "data" to "stateIn"
   *
   * @param data Command data for sending to UR5e robot
   * @param state Current sensor values information
   */
  void loopbackState(const URCommandData & data, URSensorInfo & state);

  /**
   * @brief Send control commands to the robot
   *
   * @param data Command data for sending to UR5e robot
   */
  void sendCmdData(URCommandData & data);

  /**
   * @brief The control of the robot is finished
   */
  void controlFinished();
};

} // namespace mc_rtde
