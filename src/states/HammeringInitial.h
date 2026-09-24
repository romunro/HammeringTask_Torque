#pragma once

#include <mc_control/fsm/State.h>

struct HammeringInitial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  bool button_clicked_ = false;
  double total_time_elapsed_ = 0.0;
};
