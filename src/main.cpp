#include "MCControlRtde.h"

#include <mc_rtc/config.h>

#include <fstream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace
{

/* Checking the existence of the file */
/* Return value: true if the file exists, false otherwise */
bool file_exists(const std::string& str)
{
  std::ifstream fs(str);
  return fs.is_open();
}

} // namespace

/* Main function of the interface */
int main(int argc, char * argv[])
{
 /* Set command line arguments options */
  /* Usage example: MCControlRtde -h simulation -f @ETC_PATH@/mc_rtde/mc_rtc_xxxxx.yaml */
  std::string conf_file;
  std::string host;
  po::options_description desc(std::string("MCControlRtde options"));

  // Get the configuration file path dedicated to this program
  std::string check_file = mc_rtde::CONFIGURATION_FILE;
  if(!file_exists(check_file))
  {
    check_file = "";
  }
  
  // clang-format off
  desc.add_options()
    ("help", "display help message")
    ("host,h", po::value<std::string>(&host)->default_value("ur5e"), "connection host, robot name or \"simulation\"")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(check_file), "configuration file");
  // clang-format on

  /* Parse command line arguments */
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
  po::notify(vm);
  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }
  mc_rtc::log::info("[mc_rtde] Reading additional configuration from {}", conf_file);

  /* Create global controller */
  mc_control::MCGlobalController gconfig(conf_file, nullptr);

  /* Check that the interface can work with the main controller robot */
  std::string module_name;
  module_name.resize(mc_rtde::ROBOT_NAME.size());
  std::transform(mc_rtde::ROBOT_NAME.begin(), mc_rtde::ROBOT_NAME.end(), module_name.begin(), ::tolower);
  if(gconfig.robot().name() != module_name)
  {
    mc_rtc::log::error(
        "[mc_rtde] This program can only handle '" + mc_rtde::ROBOT_NAME + "' at the moment");
    return 1;
  }

  /* Create MCControlRtde interface */
  mc_rtde::MCControlRtde mc_control_rtde(gconfig, host);

  /* Run of the robot control loop */
  mc_control_rtde.robotControl();

  mc_rtc::log::info("[mc_rtde] Terminated");

  return 0;
}
