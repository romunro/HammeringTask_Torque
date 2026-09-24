#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <ndcurves/curve_constraint.h>
#include <memory>

struct HammeringSwing : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> bspline_task_;
  std::shared_ptr<mc_tasks::VectorOrientationTask> vector_orientation_task_;

  ndcurves::curve_constraints<Eigen::Vector3d> curve_constraints_;
  Eigen::Vector3d nail_point_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_point_ = Eigen::Vector3d::Zero();

  double total_time_elapsed_ = 0.0;
  bool stop_ = false;

  double vector_error(const Eigen::Vector3d & va, const Eigen::Vector3d & vb) const;
  Eigen::Vector3d bezier_vel_from_task(const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> & task, double dt) const;
  void add_logs(mc_control::fsm::Controller & ctl);
  void rm_logs(mc_control::fsm::Controller & ctl);
};
