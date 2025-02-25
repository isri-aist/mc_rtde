#pragma once

#include <RBDyn/MultiBodyConfig.h>
#include <mc_rbdyn/Robot.h>

#include "ControlMode.h"
#include <mc_rtde/DriverBridgeRTDE.h>

namespace mc_rtde
{

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
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      q[i] = mbc.q[robot.jointIndexInMBC(i)][0];
    }
    std::cout << "control q is " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]
              << std::endl;

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
