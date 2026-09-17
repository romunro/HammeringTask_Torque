cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME HammeringTask_Torque)
set(PROJECT_DESCRIPTION "HammeringTask_Torque")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/etc/HammeringTask_Torque.yaml")
set(STATES_INSTALL_PREFIX ${MC_CONTROLLER_INSTALL_PREFIX})
configure_file(etc/HammeringTask_Torque.in.yaml "${CONFIG_OUT}" @ONLY)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc)
install(FILES ${CONFIG_OUT} DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc/)