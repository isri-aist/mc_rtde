#include <mc_rtde/DriverBridgeURModernDriver.h>

namespace mc_rtde
{
std::vector<double> DriverBridgeURModernDriver::getActualQ()
{
  auto & q = driver_->rt_interface_->robot_state_;
  state().getQActual(q_.data());
  std::cout << "Got encoders: " << q_[0] << " " << q_[1] << " " << q_[2] << " " << q_[3] << " " << q_[4] << " " << q_[5]
            << std::endl;
  // static int read = 0;
  // if(read++ == 100)
  // {
  // exit(0);
  // }
  return q_;
}

void DriverBridgeURModernDriver::sync()
{
  std::mutex msg_lock; // The values are locked for reading in the class, so
                       // just use a dummy mutex
  std::unique_lock<std::mutex> locker(msg_lock);
  while(!driver_->rt_interface_->robot_state_->getControllerUpdated())
  {
    rt_msg_cond_.wait(locker);
  }
}
} // namespace mc_rtde
