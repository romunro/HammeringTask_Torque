#include "HammeringInitial.h"
#include "../HammeringTask_Torque.h"
#include <mc_rtc/logging.h>

void HammeringInitial::configure(const mc_rtc::Configuration &)
{
}

void HammeringInitial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);

  button_clicked_ = false;
  ctl.impulsive_constraint_flag = false;
  ctl.impact_detected = false;
  ctl.params_.impulse.activation_height = 0.0;
  total_time_elapsed_ = 0.0;

  // Re-add start hammering button
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Start hammering", [this, &ctl]() {
    button_clicked_ = true;
    if(ctl.number_of_hits >= ctl.params_.impact.max_hits)
    {
      ctl.number_of_hits = 0;
    }
  }));

  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(posture)
  {
    posture->resetJointsSelector(ctl.solver());
    posture->stiffness(ctl.params_.posture.base_stiffness);
    posture->weight(ctl.params_.posture.base_weight);
    posture->damping(ctl.params_.posture.base_damping);
  }

  mc_rtc::log::info("[HammeringInitial] Entered idle state, waiting for trigger.");
}

bool HammeringInitial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  total_time_elapsed_ += ctl.solver().dt();

  if(button_clicked_ && ctl.number_of_hits < ctl.params_.impact.max_hits)
  {
    if(ctl.stabilizing_eval_norm < 0.25)
    {
      mc_rtc::log::info("[HammeringInitial] Start hammering triggered (stabilizing_eval_norm: {})", ctl.stabilizing_eval_norm);
      output("BUTTON_CLICKED");
      return true;
    }
  }

  return false;
}

void HammeringInitial::teardown(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeElement({}, "Start hammering");
}

EXPORT_SINGLE_STATE("HammeringInitial", HammeringInitial)
