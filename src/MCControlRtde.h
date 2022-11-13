#pragma once

#include <mc_control/mc_global_controller.h>
#include <condition_variable>
#include <thread>

#include "UR5eControl.h"

namespace mc_rtde
{
/**
 * @brief mc_rtc control interface for rtde robots
 */
struct MCControlRtde
{
  /**
   * @brief Interface constructor and destructor
   */
  MCControlRtde(mc_control::MCGlobalController & controller, const std::string & host);

  virtual ~MCControlRtde();

  /**
   * @brief Return a reference to the global mc_rtc controller
   */
  mc_control::MCGlobalController & controller()
  {
    return globalController_;
  }

  /**
   * @brief Run of the robot control loop
   */
  void robotControl();

private:
  void addLogEntryRobotInfo();

  /*! Global mc_rtc controller */
  mc_control::MCGlobalController & globalController_;

  /*! Connection host */
  std::string host_;

  UR5eControl * robot_;
  UR5eSensorInfo state_;
  UR5eCommandData cmdData_;
  mc_rtc::Logger logger_;
  double delay_;
};

} // namespace mc_rtde
