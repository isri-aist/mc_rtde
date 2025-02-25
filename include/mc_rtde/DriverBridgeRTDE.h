#pragma once

#include <mc_rtde/DriverBridge.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

namespace mc_rtde
{

struct DriverBridgeRTDE : public DriverBridge
{
  DriverBridgeRTDE(const std::string & ip) : DriverBridge()
  {
    ur_rtde_control_ = new ur_rtde::RTDEControlInterface(ip, 500, flags, 50002, 85);
    ur_rtde_receive_ = new ur_rtde::RTDEReceiveInterface(ip, 500, {}, false, false, 90);
  }

  std::vector<double> getActualQ() override
  {
    return ur_rtde_receive_->getActualQ();
  }

  std::vector<double> getJointTorques() override
  {
    return ur_rtde_control_->getJointTorques();
  }

  void servoJ(const std::vector<double> & q) override
  {
    auto start_t = ur_rtde_control_->initPeriod();
    ur_rtde_control_->servoJ(q, servoj_velocity, servoj_acceleration, dt, lookahead_time, servoj_gain);
    ur_rtde_control_->waitPeriod(start_t);
  }

  void speedJ(const std::vector<double> & dq) override
  {
    auto start_t = ur_rtde_control_->initPeriod();
    ur_rtde_control_->speedJ(dq, speedj_acceleration, dt);
    ur_rtde_control_->waitPeriod(start_t);
  }

  Driver driver() const noexcept override
  {
    return Driver::ur_rtde;
  }

  void sync() override {}

protected:
  uint16_t flags = ur_rtde::RTDEControlInterface::FLAG_VERBOSE | ur_rtde::RTDEControlInterface::FLAG_UPLOAD_SCRIPT;

  /* Communication information with a real robot */
  ur_rtde::RTDEControlInterface * ur_rtde_control_;
  ur_rtde::RTDEReceiveInterface * ur_rtde_receive_;

  // Parameters
  const double dt = 0.002;
  const double lookahead_time = 0.03;
  const double servoj_acceleration = 0.01;
  const double servoj_velocity = 0.05;
  const double servoj_gain = 100;
  const double speedj_acceleration = 0.5;
};

} // namespace mc_rtde
