#pragma once

#include <mc_control/Configuration.h>

namespace mc_rtde
{
struct ToolInterface
{
  virtual ~ToolInterface() = default;
  virtual void connect() = 0;
  virtual void control() = 0;

  virtual void setPosition(const std::vector<double> & position) = 0;
  virtual void setForce(const std::vector<double> & force) = 0;
  virtual void setSpeed(const std::vector<double> & speed) = 0;

  virtual std::vector<double> getPosition() = 0;
  virtual std::vector<double> getForce() = 0;
  virtual std::vector<double> getSpeed() = 0;
  virtual std::vector<double> getStatus(const std::vector<std::string> & vars) = 0;

  virtual void setState(std::vector<double> state) = 0;
  virtual void setCommand(std::vector<double> command) = 0;
  virtual int getDOF() = 0;
  virtual std::vector<double> getState() = 0;
  virtual std::vector<double> getCommand() = 0;

  virtual void autoCalibrate() = 0;
};
} // namespace mc_rtde
