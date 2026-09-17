#include "HammeringTask_Torque_Initial.h"

#include "../HammeringTask_Torque.h"

void HammeringTask_Torque_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTask_Torque_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
}

bool HammeringTask_Torque_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  output("OK");
  return true;
}

void HammeringTask_Torque_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
}

EXPORT_SINGLE_STATE("HammeringTask_Torque_Initial", HammeringTask_Torque_Initial)
