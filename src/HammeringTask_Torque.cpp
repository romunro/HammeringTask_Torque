#include "HammeringTask_Torque.h"

HammeringTask_Torque::HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("HammeringTask_Torque init done ");
}

bool HammeringTask_Torque::run()
{
  return mc_control::fsm::Controller::run();
}

void HammeringTask_Torque::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


