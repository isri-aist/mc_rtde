#pragma once

#include <ur_rtde/robotiq_gripper.h>

#include <mc_rtde/tools/ToolInterface.h>

namespace mc_rtde
{
struct ToolGripperRobotiq : public ToolInterface
{
  ToolGripperRobotiq(const std::string & ip, int port = 63352) : ip_(ip), port_(port){};
  void connect() override;

  /**
   * Applying `command_` to tool
   */
  void control() override;

  void setPosition(const std::vector<double> & position) override;
  void setForce(const std::vector<double> & force) override;
  void setSpeed(const std::vector<double> & speed) override;

  std::vector<double> getPosition() override;
  std::vector<double> getForce() override;
  std::vector<double> getSpeed() override;

  /**
   * @brief Gets the current status of the Robotiq gripper.
   *
   * For more defails, check [ur_rtde APT](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#robotiq-gripper-api)
   *
   * @param vars Accepted variable names:
   *
   * - `"ACT"` : activate (1 while activated, can be reset to clear fault status)
   *
   * - `"GTO"` : go to (will perform go to with the actions set in pos, for, spe)
   *
   * - `"ATR"` : auto-release (emergency slow move)
   *
   * - `"ARD"` : auto-release direction (open(1) or close(0) during auto-release)
   *
   * - `"FOR"` : force (0.0-1.0)
   *
   * - `"SPE"` : speed (0.0-1.0)
   *
   * - `"POS"` : position (0.0-1.0), 1.0 = open
   *
   * Read only:
   *
   * - `"STA"` : status (0 = is reset, 1 = activating, 3 = active)
   *
   * - `"PRE"` : position request (echo of last commanded position)
   *
   * - `"OBJ"` : object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
   *
   * - `"FLT"` : fault (0=ok, see manual for errors if not zero)
   *
   * @return Values in the same order as @p vars.
   * `"FOR"`, `"SPE"`, and `"POS"` is in
   * [UNIT_NORMALIZED](https://sdurobotics.gitlab.io/ur_rtde/api/api.html#_CPPv4N7ur_rtde14RobotiqGripper5eUnit15UNIT_NORMALIZEDE)
   */
  std::vector<double> getStatus(const std::vector<std::string> & vars) override;

  void setState(std::vector<double> state);
  void setCommand(std::vector<double> state);
  int getDOF();
  std::vector<double> getState();
  std::vector<double> getCommand();

  void autoCalibrate() override;

private:
  ur_rtde::RobotiqGripper * ur_rtde_gripper_;
  std::string ip_;
  int port_;
  int dof = 1;
  std::vector<double> state_ = {0};
  std::vector<double> command_ = {0};

  std::vector<double> last_sent_pos_ = command_;
  std::vector<double> last_target_pos_ = command_;
  size_t stable_count_ = 0;
  const double steady_threshold_ = 0.001f;
  const int stable_required_ = 5;

  // TOTEST: toolMutex is privately own
  mutable std::mutex stateMutex_;
  mutable std::mutex commandMutex_;
};

} // namespace mc_rtde
