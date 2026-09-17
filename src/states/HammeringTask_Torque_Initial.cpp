#include "HammeringTask_Torque_Initial.h"

#include "../HammeringTask_Torque.h"
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/logging.h>

void HammeringTask_Torque_Initial::configure(const mc_rtc::Configuration & config)
{
}

void HammeringTask_Torque_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  start_hammering_ = false;

  ctl.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this]() {
    start_hammering_ = true;
  }));

  mc_rtc::log::info("HammeringTask_Torque_Initial: LIPM stabilizer active.");
}

bool HammeringTask_Torque_Initial::run(mc_control::fsm::Controller & ctl_)
{
  if(start_hammering_)
  {
    output("START_HAMMERING");
    return true;
  }
  return false;
}

void HammeringTask_Torque_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringTask_Torque_Initial", HammeringTask_Torque_Initial)
