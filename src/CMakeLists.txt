set(PROGRAM_NAME MCControlRtde)
set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options thread)
find_package(ur_rtde REQUIRED)

set(HEADERS URControlType.h URControl.h ControlMode.h thread.h)

set(SOURCES main.cpp URControl.cpp)

add_executable(${PROGRAM_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROGRAM_NAME} PUBLIC mc_rtc::mc_control
                                             Boost::program_options)
target_link_libraries(${PROGRAM_NAME} PRIVATE ur_rtde::rtde)

install(TARGETS ${PROGRAM_NAME} DESTINATION bin)
