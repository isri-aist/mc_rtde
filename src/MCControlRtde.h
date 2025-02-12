#pragma once

#include <condition_variable>
#include <mc_control/mc_global_controller.h>
#include <thread>

#include "URControl.h"

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

  URControl * robot_;
  URSensorInfo state_;
  URCommandData cmdData_;
  mc_rtc::Logger logger_;
  double delay_;
};

} // namespace mc_rtde
