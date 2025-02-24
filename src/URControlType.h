#pragma once

#include <RBDyn/MultiBodyConfig.h>
#include <mc_rbdyn/Robot.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include "ControlMode.h"

namespace mc_rtde
{

/**
 * Provides the same interface for ur_rtde and ur_modern_driver
 * Specialize this interface for each driver
 *
 * All joint indices are specified in the robot generalized joint vector (e.g 6 dof for UR10)
 * The user is responsible for converting to/from the control robot's mbc generalized vectors.
 */
struct DriverBridge
{
  virtual std::vector<double> getActualQ() = 0;
  virtual std::vector<double> getJointTorques() = 0;
  virtual void servoJ(const std::vector<double> & q) = 0;
  virtual void speedJ(const std::vector<double> & alpha) = 0;

  virtual Driver driver() const noexcept = 0;
};

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

struct DriverBridgeURModernDriver : public DriverBridge
{
  // XXX: not implemented
  DriverBridgeURModernDriver(const std::string & ip) {}
  std::vector<double> getActualQ() override
  {
    return {};
  }
  std::vector<double> getJointTorques() override
  {
    return {};
  }
  void servoJ(const std::vector<double> & q) override {}
  void speedJ(const std::vector<double> & alpha) override {}

  Driver driver() const noexcept override
  {
    return Driver::ur_modern_driver;
  }
};

/**
 * @brief Current sensor values information of UR5e robot
 */
struct URSensorInfo
{
  URSensorInfo()
  {
    qIn_.resize(6);
    dqIn_.resize(6);
    torqIn_.resize(6);
  }
  /* Position(Angle) values */
  std::vector<double> qIn_ = std::vector<double>(6, 0.0);
  /* Velocity values */
  std::vector<double> dqIn_ = std::vector<double>(6, 0.0);
  /* Torque values */
  std::vector<double> torqIn_ = std::vector<double>(6, 0.0);
};

template<ControlMode cm>
struct URControlType
{
  static_assert(static_cast<int>(cm) == static_cast<int>(cm) + 1, "This must be specialized");
};

template<>
struct URControlType<ControlMode::Position>
{
private:
  std::vector<double> q = std::vector<double>(6, 0.0);

public:
  void control(DriverBridge & driverBridge, const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < q.size(); ++i)
    {
      q[i] = mbc.q[robot.jointIndexByName(rjo[i])][0];
    }

    driverBridge.servoJ(q);
  }
};

template<>
struct URControlType<ControlMode::Velocity>
{
private:
  std::vector<double> dq = std::vector<double>(6, 0.0);

public:
  void control(DriverBridge & driverBridge, const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < dq.size(); ++i)
    {
      dq[i] = mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
    }

    driverBridge.speedJ(dq);
  }
};

template<>
struct URControlType<ControlMode::Torque>
{
private:
  double acceleration = 0.5;
  double dt = 0.002;

public:
  void control(DriverBridge & driverBridge, const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc)
  {
    // auto start_t = robot_.initPeriod();
    // // TODO check the documentation
    // robot_.waitPeriod(start_t);
  }
};
} // namespace mc_rtde
