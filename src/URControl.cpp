
#include "URControl.h"
#include "ControlMode.h"
#include <condition_variable>
#include <ctime>
#include <exception>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <mutex>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace mc_rtde
{

struct ControlLoopDataBase
{

  ControlLoopDataBase(ControlMode cm) : cm_(cm), controller_(nullptr), ur_threads_(nullptr){};

  ControlMode cm_;
  mc_control::MCGlobalController * controller_;
  std::thread * controller_run_;
  std::condition_variable controller_run_cv_;
  std::vector<std::thread> * ur_threads_;
};

template<ControlMode cm>
struct ControlLoopData : public ControlLoopDataBase
{
  ControlLoopData() : ControlLoopDataBase(cm), urs(nullptr){};

  std::vector<URControlLoop<cm>> * urs;
};

template<ControlMode cm>
void * global_thread_init(mc_control::MCGlobalController::GlobalConfiguration & qconfig)
{
  auto rtdeConfig = qconfig.config("RTDE");

  auto loop_data = new ControlLoopData<cm>();
  loop_data->controller_ = new mc_control::MCGlobalController(qconfig);
  loop_data->ur_threads_ = new std::vector<std::thread>();
  auto & controller = *loop_data->controller_;

  if(controller.controller().timeStep < 0.002)
  {
    mc_rtc::log::error_and_throw("[mc_rtde] mc_rtc cannot run faster than 2kHz with mc_rtde");
  }

  size_t n_steps = std::ceil(controller.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("[mc_rtde] mc_rtc running at {}Hz, will interpolate every {} ur control step", freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
  }
  // Initialize controlled ur robots
  loop_data->urs = new std::vector<URControlLoop<cm>>();
  auto & urs = *loop_data->urs;

  std::vector<std::thread> ur_init_thread;
  std::mutex ur_init_mutex;
  std::condition_variable ur_init_cv;
  bool ur_init_ready = false;
  for(auto & robot : robots)
  {
    if(robot.mb().nrDof() == 0)
    {
      continue;
    }

    if(rtdeConfig.has(robot.name()))
    {
      std::string ip = rtdeConfig(robot.name())("ip");
      ur_init_thread.emplace_back(
          [&, ip]()
          {
            {
              std::unique_lock<std::mutex> lock(ur_init_mutex);
              ur_init_cv.wait(lock, [&ur_init_ready]() { return ur_init_ready; });
            }

            auto ur = std::unique_ptr<URControlLoop<cm>>(new URControlLoop<cm>(robot.name(), ip));
            std::unique_lock<std::mutex> lock(ur_init_mutex);
          });
    }
    else
    {
      mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                           robot.name());
    }
  }

  ur_init_ready = true;
  ur_init_cv.notify_all();
  for(auto & th : ur_init_thread)
  {
    th.join();
  }

  for(auto & ur : urs)
  {
    ur.init(controller);
  }

  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"RTDE"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

  // Start ur control loop
  static std::mutex startMutex;
  static std::condition_variable startCV;
  static bool startControl = false;

  for(auto & ur : urs)
  {
    loop_data->ur_threads_->emplace_back(
        [&]() { ur.controlThread(controller, startMutex, startCV, startControl, controller.running); });
  }

  startControl = true;
  startCV.notify_all();

  loop_data->controller_run_ = new std::thread(
      [loop_data]()
      {
        auto controller_ptr = loop_data->controller_;
        auto & controller = *controller_ptr;
        auto & urs_ = *loop_data->urs;
        std::mutex controller_run_mtx;
        std::timespec tv;
        clock_gettime(CLOCK_REALTIME, &tv);
        // Current time in milliseconds
        double current_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6;
        // Will record the time that passed between two runs
        double elapsed_t = 0;
        controller.controller().logger().addLogEntry("mc_franka_delay", [&elapsed_t]() { return elapsed_t; });
        while(controller.running)
        {
          std::unique_lock lck(controller_run_mtx);
          loop_data->controller_run_cv_.wait(lck);
          clock_gettime(CLOCK_REALTIME, &tv);
          elapsed_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6 - current_t;
          current_t = elapsed_t + current_t;

          // Update from ur sensors
          for(auto & ur : urs_)
          {
            ur.updateSensors(controller);
          }

          // Run the controller
          controller.run();

          // Update ur commands
          for(auto & ur : urs_)
          {
            ur.updateControl(controller);
          }
        }
      });

  return loop_data;
}

template<ControlMode cm>
void run_impl(void * data)
{
  auto control_data = static_cast<ControlLoopData<cm> *>(data);
  auto controller_ptr = control_data->controller_;
  auto & controller = *controller_ptr;
  while(controller.running)
  {
    control_data->controller_run_cv_.notify_one();
    // Sleep until the next cycle
    sched_yield();
  }
  for(auto & th : *control_data->ur_threads_)
  {
    th.join();
  }
  control_data->controller_run_->join();
  delete control_data->urs;
  delete controller_ptr;
}

void run(void * data)
{
  auto control_data = static_cast<ControlLoopDataBase *>(data);
  switch(control_data->cm_)
  {
    case ControlMode::Position:
      run_impl<ControlMode::Position>(data);
      break;
    case ControlMode::Velocity:
      run_impl<ControlMode::Velocity>(data);
      break;
    case ControlMode::Torque:
      run_impl<ControlMode::Torque>(data);
      break;
  }
}

void * init(int argc, char * argv[], uint64_t & cycle_ns)
{
  std::string conf_file = "";
  po::options_description desc("MCControlRTDE options");
  // clang-format off
   desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    std::cout << "see etc/mc_rtc_ur.yaml for ur_rtde configuration\n";
    return nullptr;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("Franka"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Franka section in the configuration, see etc/sample.yaml for an example");
  }
  auto frankaConfig = gconfig.config("Franka");
  ControlMode cm = frankaConfig("ControlMode", ControlMode::Velocity);
  bool ShowNetworkWarnings = frankaConfig("ShowNetworkWarnings", true);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        return global_thread_init<ControlMode::Position>(gconfig);
      case ControlMode::Velocity:
        return global_thread_init<ControlMode::Velocity>(gconfig);
      case ControlMode::Torque:
        return global_thread_init<ControlMode::Torque>(gconfig);
      default:
        return nullptr;
    }
  }
  catch(const std::exception & e)
  {
    std::cerr << "mc_rtde::Exception " << e.what() << "\n";
    return nullptr;
  }
}

} // namespace mc_rtde
