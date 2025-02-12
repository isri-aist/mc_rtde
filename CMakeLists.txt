cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME mc_rtde)
set(PROJECT_DESCRIPTION "RTDE robot controller for mc_rtc")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
add_subdirectory(src)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/etc/"
  DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/mc_rtde"
  FILES_MATCHING
  PATTERN "*.yaml")
