#include "HammeringRetract.h"
#include "../HammeringTask_Torque.h"
#include <mc_rtc/logging.h>

void HammeringRetract::configure(const mc_rtc::Configuration & config)
{
  if(config.has("post_impact_get_away_distance")) config("post_impact_get_away_distance", get_away_distance_);
  if(config.has("transform_task_stiffness")) config("transform_task_stiffness", stiffness_);
  if(config.has("transform_task_damping")) config("transform_task_damping", damping_);
  if(config.has("transform_task_weight")) config("transform_task_weight", weight_);
  if(config.has("posture_task_stiffness")) config("posture_task_stiffness", posture_stiffness_);
  if(config.has("posture_task_weight")) config("posture_task_weight", posture_weight_);
  if(config.has("timeout")) config("timeout", timeout_);
}

void HammeringRetract::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  total_time_elapsed_ = 0.0;

  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(posture)
  {
    posture->stiffness(posture_stiffness_);
    posture->weight(posture_weight_);
    posture->damping(ctl.params_.posture.base_damping);
  }

  Eigen::Vector3d target_velocity = -0.1 * ctl.nail_rot.transpose() * ctl.params_.trajectory.final_velocity;
  sva::MotionVecd target_vel(Eigen::Vector3d::Zero(), target_velocity);

  Eigen::Vector3d nail_point = ctl.robots().robot(ctl.params_.nail_robot_name).frame(ctl.params_.nail_frame).position().translation() + Eigen::Vector3d(0.0, 0.0, 0.2);
  Eigen::Vector3d get_away_target = nail_point + ctl.nail_normal_vector_world_frame * get_away_distance_;

  sva::PTransformd target_transform = sva::PTransformd(sva::RotX(M_PI))
      * sva::PTransformd(sva::RotY(M_PI / 2.0))
      * sva::PTransformd(ctl.robots().robot(ctl.params_.nail_robot_name).frame(ctl.params_.nail_frame).position().rotation())
      * sva::PTransformd(get_away_target);

  transform_task_ = std::make_shared<mc_tasks::TransformTask>(
      ctl.robot().frame(ctl.params_.hammer_head_frame), stiffness_, weight_);
  transform_task_->targetVel(target_vel);
  transform_task_->target(target_transform);
  transform_task_->setGains(stiffness_, damping_);

  ctl.solver().addTask(transform_task_);

  mc_rtc::log::info("[HammeringRetract] Retracting hammer {}m away from impact point.", get_away_distance_);
}

bool HammeringRetract::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  total_time_elapsed_ += ctl.solver().dt();

  if(transform_task_ && (transform_task_->eval().norm() < 0.03 || total_time_elapsed_ >= timeout_))
  {
    mc_rtc::log::info("[HammeringRetract] Retract completed (eval: {}, elapsed: {:.2f}s). Returning to idle.",
                      transform_task_->eval().norm(), total_time_elapsed_);
    output("STOP");
    return true;
  }

  return false;
}

void HammeringRetract::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  if(transform_task_)
  {
    ctl.solver().removeTask(transform_task_);
    transform_task_.reset();
  }
  mc_rtc::log::info("[HammeringRetract] Retract completed and task removed.");
}

EXPORT_SINGLE_STATE("HammeringRetract", HammeringRetract)
