#pragma once
#include <vector>

namespace mc_rtde
{

enum class Driver
{
  ur_rtde,
  ur_modern_driver
};

/**
 * Provides the same interface for ur_rtde and ur_modern_driver
 * Specialize this interface for each driver
 *
 * All joint indices are specified in the robot generalized joint vector (e.g 6 dof for UR10)
 * The user is responsible for converting to/from the control robot's mbc generalized vectors.
 */
struct DriverBridge
{
  virtual void sync() = 0;
  virtual std::vector<double> getActualQ() = 0;
  virtual std::vector<double> getJointTorques() = 0;
  virtual void servoJ(const std::vector<double> & q) = 0;
  virtual void speedJ(const std::vector<double> & alpha) = 0;

  virtual Driver driver() const noexcept = 0;
};

} // namespace mc_rtde
