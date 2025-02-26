#pragma once

#include <iostream>
#include <mc_rtde/DriverBridge.h>
#include <memory>
#include <string>
#include <ur_modern_driver/ur_driver.h>

namespace mc_rtde
{

struct DriverBridgeURModernDriver : public DriverBridge
{
  DriverBridgeURModernDriver(const std::string & ip, double cycle_s);
  ~DriverBridgeURModernDriver() override;

  std::vector<double> getActualQ() override;
  std::vector<double> getJointTorques() override
  {
    return tau_;
  }

  void servoJ(const std::vector<double> & q) override;
  void speedJ(const std::vector<double> & qd) override;

  Driver driver() const noexcept override
  {
    return Driver::ur_modern_driver;
  }

  /**
   * In order for sync to work, you must call setReadData after all data has been read
   */
  void sync() override;
  virtual void setDataRead() override
  {
    // Reset sync state
    state().setControllerUpdated();
  }

protected:
  void start();
  void stop();

  RobotStateRT & state()
  {
    return *driver_->rt_interface_->robot_state_;
  }

protected:
  std::unique_ptr<UrDriver> driver_;
  std::condition_variable rt_msg_cond_;
  std::condition_variable msg_cond_;

  double cycle_s_ = 0.008;
  double servoJTime_ = cycle_s_;
  // FIXME: hardcoded
  // What do these values mean exactly
  double serjoJLookahead_ = 0.03;
  double servoJGain_ = 300;

  std::vector<double> q_;
  std::vector<double> qd_;
  std::vector<double> tau_;
};

} // namespace mc_rtde
