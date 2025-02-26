#include <mc_rtde/DriverBridgeURModernDriver.h>

namespace mc_rtde
{
std::vector<double> DriverBridgeURModernDriver::getActualQ()
{
  auto & q = driver_->rt_interface_->robot_state_;
  state().getQActual(q_.data());
  // std::cout << "Got encoders: " << q_[0] << " " << q_[1] << " " << q_[2] << " " << q_[3] << " " << q_[4] << " " <<
  // q_[5] << std::endl; static int read = 0; if(read++ == 50)
  // {
  // exit(0);
  // }
  return q_;
}

void DriverBridgeURModernDriver::servoJ(const std::vector<double> & q)
{
  std::cout << "Send servoJ: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]
            << std::endl;
  driver_->servoj(q.data());
}
void DriverBridgeURModernDriver::speedJ(const std::vector<double> & qd)
{
  // std::cout << "Would speedJ: " << qd[0] << " " << qd[1] << " " << qd[2] << " " << qd[3] << " " << qd[4] << " "
  //           << qd[5] << std::endl;

  driver_->setPayload(0);
  driver_->setMinPayload(0);
  driver_->setMaxPayload(10);
  double acc = 100;
  // driver_->setSpeed(qd.data(), acc);
  std::vector<double> dummy_qd = {0., 0., 0., 0., -0.05, 0.};
  dummy_qd[4] = qd[4];
  std::cout << qd[4] << std::endl;
  driver_->setSpeed(dummy_qd.data(), acc);
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
