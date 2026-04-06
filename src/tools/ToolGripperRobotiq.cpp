#include <mc_rtde/tools/ToolGripperRobotiq.h>

namespace mc_rtde
{

void ToolGripperRobotiq::connect()
{
  ur_rtde_gripper_ = new ur_rtde::RobotiqGripper(ip_, port_, false);

  ur_rtde_gripper_->connect();
  ur_rtde_gripper_->activate();
  ur_rtde_gripper_->setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_NORMALIZED);

  // No need for mutex protection since gripperThread() has not been created
  state_ = {0.725 * (1.0 - getPosition()[0])};
}

void ToolGripperRobotiq::control()
{
  auto vectorDiff = [](const std::vector<double> & a, const std::vector<double> & b, double threshold) -> bool
  {
    for(size_t i = 0; i < a.size(); i++)
    {
      if(std::abs(a[i] - b[i]) > threshold) return true;
    }
    return false;
  };

  std::lock_guard<std::mutex> lock(commandMutex_);
  if(vectorDiff(command_, last_target_pos_, steady_threshold_))
  {
    stable_count_ = 0;
    last_target_pos_ = command_;
  }
  else
  {
    stable_count_++;
  }

  if(stable_count_ >= stable_required_ && vectorDiff(command_, last_sent_pos_, steady_threshold_))
  {
    setPosition(command_);
    last_sent_pos_ = command_;
  }
}

void ToolGripperRobotiq::setPosition(const std::vector<double> & position)
{
  auto target = std::clamp(position[0], 0.0, 0.725);
  target = -target / 0.725 + 1.0; // maping [0, 0.725] to [1.0, 0.0]
  ur_rtde_gripper_->move(target);
}

void ToolGripperRobotiq::setForce(const std::vector<double> & force)
{
  ur_rtde_gripper_->setForce(force[0]);
}

void ToolGripperRobotiq::setSpeed(const std::vector<double> & speed)
{
  ur_rtde_gripper_->setSpeed(speed[0]);
}

std::vector<double> ToolGripperRobotiq::getPosition()
{
  return {ur_rtde_gripper_->getCurrentPosition()};
}

std::vector<double> ToolGripperRobotiq::getForce()
{
  return {(double)ur_rtde_gripper_->getVar("FOR") / 255.0};
}

std::vector<double> ToolGripperRobotiq::getSpeed()
{
  return {(double)ur_rtde_gripper_->getVar("SPE") / 255.0};
}

std::vector<double> ToolGripperRobotiq::getStatus(const std::vector<std::string> & vars)
{
  std::vector<double> values;
  values.reserve(vars.size());

  for(const auto & var : vars)
  {
    if(var == "FOR")
    {
      values.push_back(getForce()[0]);
    }
    else if(var == "SPE")
    {
      values.push_back(getSpeed()[0]);
    }
    else if(var == "POS")
    {
      values.push_back(getPosition()[0]);
    }
    else
    {
      values.push_back(static_cast<double>(ur_rtde_gripper_->getVar(var)));
    }
  }
  return values;
}

void ToolGripperRobotiq::autoCalibrate()
{
  ur_rtde_gripper_->autoCalibrate();
}

int ToolGripperRobotiq::getDOF()
{
  return dof;
}

void ToolGripperRobotiq::setState(std::vector<double> state)
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  // state_ = state;
  state_ = {0.725 * (1.0 - state[0])};
};

void ToolGripperRobotiq::setCommand(std::vector<double> command)
{
  std::lock_guard<std::mutex> lock(commandMutex_);
  command_ = command;
};

std::vector<double> ToolGripperRobotiq::getState()
{
  std::lock_guard<std::mutex> lock(stateMutex_);
  return state_;
}

std::vector<double> ToolGripperRobotiq::getCommand()
{
  std::lock_guard<std::mutex> lock(commandMutex_);
  return command_;
}

} // namespace mc_rtde
