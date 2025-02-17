#pragma once

#include <mc_rbdyn/Robot.h>
#include <ur_rtde/rtde_control_interface.h>

#include "ControlMode.h"

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
  std::vector<double> qIn_;
  /* Velocity values */
  std::vector<double> dqIn_;
  /* Torque values */
  std::vector<double> torqIn_;
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
  // Parameters
  const double dt = 0.002;
  const double lookahead_time = 0.001;
  const double acceleration = 0.5;
  const double velocity = 0.5;
  const double gain = 300;

  URSensorInfo state_;

public:
  URControlType(const URSensorInfo & state) : state_(state){};

  void update(const URSensorInfo & state)
  {
    state_ = state;
  }

  void control(ur_rtde::RTDEControlInterface & robot_)
  {
    auto start_t = robot_.initPeriod();
    robot_.servoJ(state_.qIn_, velocity, acceleration, dt, lookahead_time, gain);
    robot_.waitPeriod(start_t);
  }
};

template<>
struct URControlType<ControlMode::Velocity>
{
private:
  double acceleration = 0.5;
  double dt = 0.002;

  URSensorInfo state_;

public:
  URControlType(const URSensorInfo & state) : state_(state){};

  void update(const URSensorInfo & state)
  {
    state_ = state;
  }

  void control(ur_rtde::RTDEControlInterface & robot_)
  {
    auto start_t = robot_.initPeriod();
    robot_.speedJ(state_.dqIn_, acceleration, dt);
    robot_.waitPeriod(start_t);
  }
};

template<>
struct URControlType<ControlMode::Torque>
{
private:
  double acceleration = 0.5;
  double dt = 0.002;

  URSensorInfo state_;

public:
  URControlType(const URSensorInfo & state) : state_(state){};

  void update(const URSensorInfo & state)
  {
    state_ = state;
  }

  void control(ur_rtde::RTDEControlInterface & robot_)
  {
    auto start_t = robot_.initPeriod();
    // TODO check the documentation
    robot_.waitPeriod(start_t);
  }
};
} // namespace mc_rtde
