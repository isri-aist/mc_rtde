
#include "URControl.h"
#include "ControlMode.h"
#include <condition_variable>
#include <ctime>
#include <exception>
#include <iostream>
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
  std::thread * controller_run_ = nullptr;
  std::condition_variable controller_run_cv_;
  std::vector<std::thread> * ur_threads_;
};

template<ControlMode cm>
struct ControlLoopData : public ControlLoopDataBase
{
  ControlLoopData() : ControlLoopDataBase(cm), urs(nullptr){};

  std::vector<URControlLoopPtr<cm>> * urs;
};

template<ControlMode cm>
void * global_thread_init(mc_control::MCGlobalController::GlobalConfiguration & qconfig, const bool & interrupt)
{
  auto rtdeConfig = qconfig.config("RTDE");

  auto loop_data = new ControlLoopData<cm>();
  loop_data->controller_ = new mc_control::MCGlobalController(qconfig);
  loop_data->ur_threads_ = new std::vector<std::thread>();
  auto & controller = *loop_data->controller_;

  double cycle_s = rtdeConfig("RobotTimestep");
  double controller_s = controller.controller().timeStep;
  size_t cycle_ns = cycle_s * 1e9;
  size_t controller_ns = controller_s * 1e9;
  if(controller_ns < cycle_ns)
  {
    mc_rtc::log::error_and_throw(
        "[mc_rtde] mc_rtc cannot run faster than the robot's control frequency (RobotTimeStep= {}ms, Timestep={}ms)",
        cycle_s, controller_s);
  }
  if(controller_ns % cycle_ns != 0)
  {
    mc_rtc::log::error_and_throw("[mc_rtde] mc_rtc timestep must be a multiple of the robot's control loop frequency "
                                 "(RobotTimeStep= {}ms, Timestep={}ms)",
                                 cycle_s, controller_s);
  }

  size_t n_steps = controller_ns / cycle_ns;
  size_t freq = std::ceil(1 / controller_s);
  size_t robot_freq = std::ceil(1 / cycle_s);
  mc_rtc::log::info(
      "[mc_rtde] mc_rtc running at {}Hz, robot running at {}Hz, will compute commands every {} robot control step",
      freq, robot_freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
  }
  // Initialize controlled ur robots
  loop_data->urs = new std::vector<URControlLoopPtr<cm>>();
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
      auto driverName = rtdeConfig(robot.name())("driver", std::string{"ur_rtde"});
      auto driver = (driverName == "ur_rtde") ? Driver::ur_rtde : Driver::ur_modern_driver;
      ur_init_thread.emplace_back(
          [&, ip]()
          {
            {
              std::unique_lock<std::mutex> lock(ur_init_mutex);
              ur_init_cv.wait(lock, [&ur_init_ready]() { return ur_init_ready; });
            }

            auto ur = std::unique_ptr<URControlLoop<cm>>(new URControlLoop<cm>(driver, robot.name(), ip, cycle_s));
            std::unique_lock<std::mutex> lock(ur_init_mutex);
            urs.emplace_back(std::move(ur));
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
    ur->init(controller);
  }

  controller.init(robots.robot().encoderValues());
  std::vector<double> qIn(6, 0.0);
  for(int i = 0; i < 6; ++i)
  {
    qIn[i] = robots.robot().mbc().q[robots.robot().jointIndexInMBC(i)][0];
  }
  std::cout << "qIn is " << qIn[0] << " " << qIn[1] << " " << qIn[2] << " " << qIn[3] << " " << qIn[4] << " " << qIn[5]
            << std::endl;
  controller.running = true;
  controller.controller().gui()->addElement(
      {"RTDE"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));

  // Start ur control loop
  static std::mutex startMutex;
  static std::condition_variable startCV;
  static bool startControl = false;

  for(auto & ur : urs)
  {
    // Create UR control threads but don't run them yet, wait on startCV
    // until the first control command has been computed (by MCGlobalController::run)
    loop_data->ur_threads_->emplace_back(
        [&]() { ur->controlThread(controller, startMutex, startCV, startControl, controller.running); });
  }

  // Create main mc_rtc control thread:
  // - get the latest sensor readings from the robots
  // - run mc_rtc controller (MCGlobalController::run)
  // - sends the latest command
  //
  // This thread runs at the same frequency as the low-level control (cycle_ns).
  // However, the mc_rtc control loop can run at a lower frequency.
  // This frequency must be a multiple (n_steps) of cycle_ns as lower frequencies
  // are simply achieved by calling MCGlobalController every n_steps iterations of the low-level control loop
  loop_data->controller_run_ = new std::thread(
      [loop_data, n_steps, &interrupt]()
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
        controller.controller().logger().addLogEntry("mc_rtde_delay", [&elapsed_t]() { return elapsed_t; });

        size_t n_iter = n_steps;

        while(controller.running)
        {
          std::unique_lock lck(controller_run_mtx);
          loop_data->controller_run_cv_.wait(lck);
          if(interrupt)
          {
            std::cout << "controller_run_ interrupted" << std::endl;
            controller.running = false;
            return;
          }
          clock_gettime(CLOCK_REALTIME, &tv);
          elapsed_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6 - current_t;
          current_t = elapsed_t + current_t;

          // Update from the latest available ur sensors (non blocking)
          for(auto & ur : urs_)
          {
            ur->updateSensors(controller);
          }

          if(n_iter % n_steps == 0)
          {
            // Run the controller
            // std::cout << "Running the controller at iter " << n_iter << std::endl;
            controller.run();
          }

          // Wait until the first command has been computed to start the robot's control loop
          startControl = true;
          startCV.notify_all();

          // Update ur commands (non blocking)
          // If mc_rtc is running at a lower frequency than the low-level control, then intermediate commands
          // will need to be interpolated (especially for position control)
          for(auto & ur : urs_)
          {
            ur->updateControl(controller);
          }
          n_iter++;
        }
      });

  return loop_data;
}

template<ControlMode cm>
void run_impl(void * data, const bool & interrupt)
{
  auto control_data = static_cast<ControlLoopData<cm> *>(data);
  auto controller_ptr = control_data->controller_;
  auto & controller = *controller_ptr;
  if(interrupt)
  {
    controller.running = false;
  }
  while(controller.running)
  {
    control_data->controller_run_cv_.notify_one();

    // Sleep until the next cycle
    sched_yield();
  }
  std::cout << "Waiting for the ur control threads to stop" << std::endl;
  for(auto & th : *control_data->ur_threads_)
  {
    th.join();
  }
  std::cout << "Waiting for the main control thread to stop" << std::endl;
  control_data->controller_run_cv_.notify_one();
  control_data->controller_run_->join();
  std::cout << "Main control thread stopped" << std::endl;
  control_data->urs->clear();
  delete control_data->urs;
  delete controller_ptr;
}

void run(void * data, const bool & interrupt)
{
  auto control_data = static_cast<ControlLoopDataBase *>(data);
  switch(control_data->cm_)
  {
    case ControlMode::Position:
      run_impl<ControlMode::Position>(data, interrupt);
      break;
    case ControlMode::Velocity:
      run_impl<ControlMode::Velocity>(data, interrupt);
      break;
    case ControlMode::Torque:
      run_impl<ControlMode::Torque>(data, interrupt);
      break;
  }
}

void * init(int argc, char * argv[], uint64_t & cycle_ns, const bool & interrupt)
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
  if(!gconfig.config.has("RTDE"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No RTDE section in the configuration, see etc/mc_rtc_ur.yaml for an example");
  }
  auto urConfig = gconfig.config("RTDE");
  ControlMode cm = urConfig("ControlMode", ControlMode::Position);
  auto driverConfig = urConfig("Driver", std::string{"ur_rtde"});
  Driver driver = (driverConfig == "ur_rtde") ? Driver::ur_rtde : Driver::ur_modern_driver;
  if(urConfig.has("RobotTimestep"))
  {
    cycle_ns = static_cast<double>(urConfig("RobotTimestep")) * 1e9;
  }
  else
  {
    urConfig.add("RobotTimestep", cycle_ns * 1e-9);
  }
  auto cycle_s = cycle_ns * 1e-9;
  if(cycle_s < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("RobotTimestep must be at least 1ms");
  }
  mc_rtc::log::info("RobotTimestep is: {}s (frequency={:.2f}Hz), the RT thread will run at this frequency", cycle_s,
                    1 / cycle_s);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        return global_thread_init<ControlMode::Position>(gconfig, interrupt);
      case ControlMode::Velocity:
        return global_thread_init<ControlMode::Velocity>(gconfig, interrupt);
      case ControlMode::Torque:
        return global_thread_init<ControlMode::Torque>(gconfig, interrupt);
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
