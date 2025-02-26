#include <iostream>
#include <mc_rtde/DriverBridgeURModernDriver.h>

namespace mc_rtde
{

// XXX: not implemented
DriverBridgeURModernDriver::DriverBridgeURModernDriver(const std::string & ip, double cycle_s)
: q_(6, 0.0), qd_(6, 0.0), tau_(6, 0.0), cycle_s_(cycle_s)
{
  driver_ = std::make_unique<UrDriver>(rt_msg_cond_, msg_cond_, ip);
  driver_->start();
  // For position control
  driver_->uploadProg();
}

DriverBridgeURModernDriver::~DriverBridgeURModernDriver()
{
  stop();
}

std::vector<double> DriverBridgeURModernDriver::getActualQ()
{
  auto & q = driver_->rt_interface_->robot_state_;
  state().getQActual(q_.data());
  // std::cout << "Got encoders: " << q_[0] << " " << q_[1] << " " << q_[2] << " " << q_[3] << " " << q_[4] << " " <<
  // q_[5]
  //           << std::endl;
  return q_;
}

void DriverBridgeURModernDriver::servoJ(const std::vector<double> & q)
{
  // std::cout << "Send servoJ: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]
  //           << std::endl;
  // std::cout << "Parameters: " << servoJTime_ << " " << serjoJLookahead_ << " " << servoJGain_ << std::endl;

  driver_->setServojTime(servoJTime_);
  driver_->setServojLookahead(serjoJLookahead_);
  driver_->setServojGain(servoJGain_);
  driver_->servoj(q.data());
}

void DriverBridgeURModernDriver::speedJ(const std::vector<double> & qd)
{
  // std::cout << "Would speedJ: " << qd[0] << " " << qd[1] << " " << qd[2] << " " << qd[3] << " " << qd[4] << " "
  //           << qd[5] << std::endl;

  // XXX: hardcoded payload
  driver_->setPayload(0);
  driver_->setMinPayload(0);
  driver_->setMaxPayload(10);
  double acc = 100;
  driver_->setSpeed(qd.data(), acc);
  // std::vector<double> dummy_qd = {0., 0., 0., 0., 0.05, 0.};
  // dummy_qd[4] = qd[4];
  // std::cout << qd[4] << std::endl;
  // driver_->setSpeed(dummy_qd.data(), acc);
}

void DriverBridgeURModernDriver::sync()
{
  std::mutex msg_lock;
  std::unique_lock<std::mutex> lock(msg_lock);
  rt_msg_cond_.wait(lock, [this]() { return driver_->rt_interface_->robot_state_->getControllerUpdated(); });
}

void DriverBridgeURModernDriver::start()
{
  if(not driver_->start())
  {
    throw std::runtime_error("Cannot start the UR robot");
  }
  sync();
}

void DriverBridgeURModernDriver::stop()
{
  std::cout << "STOP" << std::endl;
  std::vector<double> dummy_qd = {0., 0., 0., 0., 0., 0.};
  driver_->setSpeed(dummy_qd.data());

  driver_->halt();
}
} // namespace mc_rtde
