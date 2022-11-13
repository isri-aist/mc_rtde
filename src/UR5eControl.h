#ifndef _UR5ECONTROL_H_
#define _UR5ECONTROL_H_

#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <map>

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "ControlMode.h"

namespace mc_rtde
{
  const std::string ROBOT_NAME = "UR5e";
  const std::string CONFIGURATION_FILE = "/usr/local/etc/mc_rtde/mc_rtc_ur5e.yaml";
  constexpr int UR5E_JOINT_COUNT = 6;

/**
 * @brief Configuration file parameters for mc_rtde
 */
struct UR5eConfigParameter
{
  UR5eConfigParameter()
  : cm_(ControlMode::Position), joint_speed_(1.05), joint_acceleration_(1.4), targetIP_("localhost")
  {}
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
struct UR5eSensorInfo
{
  UR5eSensorInfo()
  {
    qIn_.resize(UR5E_JOINT_COUNT);
    dqIn_.resize(UR5E_JOINT_COUNT);
    torqIn_.resize(UR5E_JOINT_COUNT);
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
struct UR5eCommandData
{
  UR5eCommandData()
  {
    qOut_.resize(UR5E_JOINT_COUNT);
    dqOut_.resize(UR5E_JOINT_COUNT);
    torqOut_.resize(UR5E_JOINT_COUNT);
    for(int i = 0; i < UR5E_JOINT_COUNT; ++i)
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
class UR5eControl
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
  UR5eSensorInfo stateIn_;

public:
  /**
   * @brief Interface constructor and destructor
   * ur_rtde connection with robot using specified parameters.
   *
   * @param config_param Configuration file parameters
   */
  UR5eControl(const UR5eConfigParameter & config_param);

  /**
   * @brief Interface constructor and destructor
   * Running simulation only. No connection to real robot.
   *
   * @param host "simulation" only
   * @param config_param Configuration file parameters
   */
  UR5eControl(const std::string & host, const UR5eConfigParameter & config_param);

  ~UR5eControl()
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
  bool getState(UR5eSensorInfo & state);

  /**
   * @brief Set the start state values for simulation
   * 
   * @param stance Value defined by RobotModule
   * @param state Current sensor values information
   */
  void setStartState(const std::map<std::string, std::vector<double>> & stance, UR5eSensorInfo & state);

  /**
   * @brief Loop back the value of "data" to "stateIn"
   * 
   * @param data Command data for sending to UR5e robot
   * @param state Current sensor values information
   */
  void loopbackState(const UR5eCommandData & data, UR5eSensorInfo & state);

  /**
   * @brief Send control commands to the robot
   * 
   * @param data Command data for sending to UR5e robot
   */
  void sendCmdData(UR5eCommandData & data);

  /**
   * @brief The control of the robot is finished
   */
  void controlFinished();
};

} // namespace mc_rtde
#endif
