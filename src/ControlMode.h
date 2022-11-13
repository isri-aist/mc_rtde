#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rtde
{

enum class ControlMode
{
  Position,
  Velocity,
  Torque
};
}

namespace mc_rtc
{

template<>
struct ConfigurationLoader<mc_rtde::ControlMode>
{
  static Configuration save(const mc_rtde::ControlMode & cm)
  {
    Configuration c;
    switch(cm)
    {
      case mc_rtde::ControlMode::Position:
        c.add("cm", "Position");
        break;
      case mc_rtde::ControlMode::Velocity:
        c.add("cm", "Velocity");
        break;
      case mc_rtde::ControlMode::Torque:
        c.add("cm", "Torque");
        break;
      default:
        log::error_and_throw<std::runtime_error>("ControlMode has unexpected value");
    }
    return c("cm");
  }

  static mc_rtde::ControlMode load(const Configuration & conf)
  {
    std::string cm = conf;
    if(cm == "Position")
    {
      return mc_rtde::ControlMode::Position;
    }
    if(cm == "Velocity")
    {
      return mc_rtde::ControlMode::Velocity;
    }
    if(cm == "Torque")
    {
      return mc_rtde::ControlMode::Torque;
    }
    log::error_and_throw<std::runtime_error>("ControlMode has unexpected value {}", cm);
  }
};

} // namespace mc_rtc
