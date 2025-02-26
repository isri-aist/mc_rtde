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

# CAP_SYS_NICE is required to set the SCHED_DEADLINE policy, we set it on the generated
# executable and the installed one
set(JOB_POOL_OPTION)
if(${CMAKE_VERSION} VERSION_GREATER "3.14")
  set(JOB_POOL_OPTION JOB_POOL console)
endif()
add_custom_target(
  setcap_MCFrankaControl ALL
  COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip $<TARGET_FILE:${PROGRAM_NAME}>
  DEPENDS MCFrankaControl ${JOB_POOL_OPTION}
)

install(TARGETS ${PROGRAM_NAME} DESTINATION bin)
install(
  CODE "execute_process(COMMAND ${SUDO} -S ${SETCAP} cap_sys_nice+eip ${CMAKE_INSTALL_PREFIX}/bin/${PROGRAM_NAME})"
)
