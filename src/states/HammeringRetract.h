#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <memory>

struct HammeringRetract : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::TransformTask> transform_task_;
  double total_time_elapsed_ = 0.0;

  double get_away_distance_ = 0.01;
  double stiffness_ = 30.0;
  double damping_ = 25.0;
  double weight_ = 100.0;
  double posture_stiffness_ = 1.0;
  double posture_weight_ = 10.0;
  double timeout_ = 2.0;
};
