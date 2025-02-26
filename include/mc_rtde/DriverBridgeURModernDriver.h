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
  // XXX: not implemented
  DriverBridgeURModernDriver(const std::string & ip) : rt_state_(msg_cond_), q_(6, 0.0), qd_(6, 0.0), tau_(6, 0.0)
  {
    driver_ = std::make_unique<UrDriver>(rt_msg_cond_, msg_cond_, ip);
    driver_->start();
  }

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

  void sync() override;

protected:
  void start()
  {
    if(not driver_->start())
    {
      throw std::runtime_error("Cannot start the UR robot");
    }
    sync();
  }
  void stop()
  {
    driver_->halt();
  }

  RobotStateRT & state()
  {
    return *driver_->rt_interface_->robot_state_;
  }

protected:
  std::unique_ptr<UrDriver> driver_;
  std::condition_variable rt_msg_cond_;
  std::condition_variable msg_cond_;
  RobotStateRT rt_state_;

  std::vector<double> q_;
  std::vector<double> qd_;
  std::vector<double> tau_;

public:
  //! \brief Available command modes
  enum CommandMode
  {
    //! \brief Can be used to only monitor the robot state without moving
    //! the robot
    Monitor,
    //! \brief Control the robot by sending joint position commands
    JointPositionControl,
    //! \brief Control the robot by sending joint velcoity commands
    JointVelocityControl,
  };
};

} // namespace mc_rtde
