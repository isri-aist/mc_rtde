set(PROGRAM_NAME MCControlRtde)
set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

set(HEADERS URControlType.h URControl.h ControlMode.h thread.h)

set(SOURCES main.cpp URControl.cpp DriverBridgeURModernDriver.cpp)

add_executable(${PROGRAM_NAME} ${SOURCES} ${HEADERS})
target_include_directories(
  ${PROGRAM_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROGRAM_NAME} PUBLIC mc_rtc::mc_control Boost::program_options)
target_link_libraries(${PROGRAM_NAME} PUBLIC ur_rtde::rtde)
target_link_libraries(${PROGRAM_NAME} PUBLIC ur_modern_driver::ur_modern_driver)

install(TARGETS ${PROGRAM_NAME} DESTINATION bin)
