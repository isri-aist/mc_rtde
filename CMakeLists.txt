cmake_minimum_required(VERSION 3.5)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_rtde)
set(PROJECT_DESCRIPTION "RTDE robot controller for mc_rtc")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)

find_package(ur_rtde REQUIRED) # modern RTDE driver supported from UR CB3
find_package(ur_modern_driver REQUIRED)

find_program(SUDO sudo)
if(NOT SUDO)
  message(FATAL_ERROR "sudo must be available on the system")
else()
  message("-- sudo command: ${SUDO}")
endif()

find_program(SETCAP setcap)
if(NOT SETCAP)
  message(FATAL_ERROR "setcap must be available on the system")
else()
  message("-- setcap command: ${SETCAP}")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
add_subdirectory(src)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/etc/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/mc_rtde"
  FILES_MATCHING
  PATTERN "*.yaml")
