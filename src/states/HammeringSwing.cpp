#include "HammeringSwing.h"
#include "../HammeringTask_Torque.h"
#include <mc_rtc/logging.h>
#include <cmath>

void HammeringSwing::configure(const mc_rtc::Configuration &)
{
}

void HammeringSwing::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  stop_ = false;
  total_time_elapsed_ = 0.0;
  ctl.impulsive_constraint_flag = false;
  ctl.impact_detected = false;
  ctl.params_.impulse.activation_height = 0.0;

  // Add stop button
  ctl.gui()->addElement({}, mc_rtc::gui::Button(ctl.params_.stop_hammering_button_name, [this]() { stop_ = true; }));

  // 1. BSpline curve constraints
  curve_constraints_.init_vel = ctl.params_.trajectory.init_velocity;
  curve_constraints_.end_vel = ctl.nail_rot.transpose() * ctl.params_.trajectory.final_velocity;

  nail_point_ = ctl.params_.trajectory.nail_target_position;
  end_point_ = nail_point_;

  Eigen::Vector3d current_pos = ctl.robot().frame(ctl.params_.hammer_head_frame).position().translation();
  mc_trajectory::BSpline::waypoints_t posWp = {
    (current_pos + nail_point_) / 2.0 + ctl.nail_normal_vector_world_frame * ctl.params_.trajectory.waypoint_height
  };
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp = {};

  sva::PTransformd target = sva::PTransformd(sva::RotX(M_PI))
      * sva::PTransformd(sva::RotY(M_PI / 2.0))
      * sva::PTransformd(ctl.robots().robot(ctl.params_.nail_robot_name).frame(ctl.params_.nail_frame).position().rotation())
      * sva::PTransformd(end_point_);

  mc_rtc::log::info("[HammeringSwing] Creating BSpline task: duration = {}s, weight = {}, stiffness = {}",
                    ctl.params_.trajectory.duration, ctl.params_.trajectory.weight, ctl.params_.trajectory.stiffness);

  bspline_task_ = std::make_shared<mc_tasks::BSplineTrajectoryTask>(
      ctl.robot().frame(ctl.params_.hammer_head_frame),
      ctl.params_.trajectory.duration,
      ctl.params_.trajectory.stiffness,
      ctl.params_.trajectory.weight,
      target,
      curve_constraints_,
      posWp,
      oriWp);

  bspline_task_->setGains(ctl.params_.trajectory.stiffness, ctl.params_.trajectory.damping);
  bspline_task_->dimWeight(ctl.params_.trajectory.dimweights);
  ctl.solver().addTask(bspline_task_);
  ctl.bspline_active_ = true;

  // 2. Posture regularization: keep full posture stiffness/weight on legs & torso for balance,
  // and only relax the left arm joints in the nullspace using dimWeight
  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(posture)
  {
    posture->stiffness(ctl.params_.posture.base_stiffness);
    posture->weight(ctl.params_.posture.base_weight);
    posture->damping(ctl.params_.posture.base_damping);

    std::vector<std::string> left_arm_joints = {
      "LSC", "LSP", "LSR", "LSY", "LEP", "LWRY", "LWRR", "LWRP", "LHDY"
    };
    Eigen::VectorXd dimW = posture->dimWeight();
    dimW.setOnes();
    const auto & robot = ctl.robot();
    auto dofOffset = robot.mb().joint(0).dof();
    double arm_dim_weight = (ctl.params_.posture.arm_nullspace_weight > 0.0 && ctl.params_.posture.arm_nullspace_weight <= 0.1)
                              ? ctl.params_.posture.arm_nullspace_weight
                              : 0.05;
    for(const auto & j : left_arm_joints)
    {
      if(robot.hasJoint(j))
      {
        auto jIndex = static_cast<int>(robot.jointIndexByName(j));
        const auto & joint = robot.mb().joint(jIndex);
        if(joint.dof() == 6) continue;
        auto dofIndex = robot.mb().jointPosInDof(jIndex) - dofOffset;
        dimW.segment(dofIndex, joint.dof()).setConstant(arm_dim_weight);
      }
    }
    posture->dimWeight(dimW);
  }

  // 3. Vector orientation task to align hammer head normal
  vector_orientation_task_ = std::make_shared<mc_tasks::VectorOrientationTask>(
      ctl.robot().frame(ctl.params_.hammer_head_frame),
      ctl.normal_vector_to_align_in_hammerhead_frame);
  vector_orientation_task_->targetVector(-ctl.nail_normal_vector_world_frame);
  vector_orientation_task_->weight(ctl.params_.vector_orientation.weight);
  vector_orientation_task_->stiffness(ctl.params_.vector_orientation.stiffness);
  vector_orientation_task_->damping(ctl.params_.vector_orientation.damping);
  ctl.solver().addTask(vector_orientation_task_);

  // 4. Impulse constraint instantiation
  if(ctl.params_.impulse.linear_impulsive_torque_ctr_flag)
  {
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(
        bspline_task_, ctl.robots(), ctl.robot().robotIndex(),
        ctl.robot().frame(ctl.params_.hammer_head_frame), ctl.nail_normal_vector_world_frame,
        ctl.params_.impulse.lambda_high, ctl.params_.impulse.lambda_low,
        ctl.params_.impulse.delta_t, ctl.params_.impulse.c_res,
        ctl.params_.impulse.limit_multiplier, ctl.logger(),
        ctl.params_.impulse.tau_high_multiplier, ctl.params_.impulse.K,
        &ctl.params_.impulse.activation_height);
    mc_rtc::log::info("[HammeringSwing] Linear impulsive torque constraint created.");
  }
  else
  {
    ctl.impulseConstraint = std::make_unique<mc_solver::ImpulseConstraint>(
        ctl.robots(), ctl.robot().robotIndex(),
        ctl.robot().frame(ctl.params_.hammer_head_frame), ctl.nail_normal_vector_world_frame,
        ctl.params_.impulse.lambda_high, ctl.params_.impulse.lambda_low,
        ctl.params_.impulse.delta_t, ctl.params_.impulse.c_res,
        ctl.params_.impulse.limit_multiplier, ctl.logger());
    mc_rtc::log::info("[HammeringSwing] Constant impulsive torque constraint created.");
  }
  ctl.impulseConstraint->filterCutoffPeriod(ctl.filter_cutoff_period_);

  add_logs(ctl_);
}

bool HammeringSwing::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  total_time_elapsed_ += ctl.solver().dt();

  if(bspline_task_)
  {
    ctl.hammer_tip_reference_velocity_vector = bezier_vel_from_task(bspline_task_, ctl.solver().dt());
    ctl.hammer_tip_reference_position_vector = bspline_task_->target().translation();
    ctl.bspline_eval = bspline_task_->evalTracking();
    ctl.bspline_eval_norm = bspline_task_->evalTracking().norm();
  }

  ctl.projected_momentum_of_hammer_tip = ctl.effective_mass * ctl.hammer_tip_actual_velocity_vector.dot(ctl.nail_normal_vector_world_frame);
  ctl.bspline_tracking_error = ctl.hammer_tip_actual_position_vector - ctl.hammer_tip_reference_position_vector;

  // Activate impulse constraint on the downswing when reference velocity points along the nail normal
  if(ctl.hammer_tip_reference_velocity_vector.dot(ctl.nail_normal_vector_world_frame) <= 0.0 && !ctl.impulsive_constraint_flag)
  {
    ctl.impulsive_constraint_flag = true;
    if(ctl.params_.impulse.activation_height <= 0.0)
    {
      ctl.params_.impulse.activation_height = (ctl.robot().frame(ctl.params_.hammer_head_frame).position().translation() -
                                               bspline_task_->target().translation()).norm();
    }
    ctl.solver().addConstraintSet(ctl.impulseConstraint);
    mc_rtc::log::info("[HammeringSwing] Downswing detected, added impulse constraint to solver. Activation height = {}",
                      ctl.params_.impulse.activation_height);
  }

  Eigen::Matrix3d current_rot = ctl.robot().frame(ctl.params_.hammer_head_frame).position().rotation();
  ctl.vector_orientation_error = vector_error(-ctl.nail_normal_vector_world_frame,
                                              (current_rot.transpose() * ctl.normal_vector_to_align_in_hammerhead_frame).normalized());

  // Impact detection
  double hand_force = ctl.robot().forceSensor("LeftHandForceSensor").force().norm();
  ctl.impact_detected = (hand_force >= ctl.params_.impact.sensor_force_threshold);

  bool stop_height_flag = (ctl.hammer_tip_actual_position_vector.z() < ctl.params_.trajectory.nail_target_position.z());

  if(ctl.impact_detected)
  {
    ctl.number_of_hits++;
    mc_rtc::log::info("[HammeringSwing] Impact detected! (Force: {} N, Hit count: {})", hand_force, ctl.number_of_hits);
    output("STOP");
    return true;
  }

  if(total_time_elapsed_ > ctl.params_.trajectory.duration && stop_height_flag)
  {
    ctl.number_of_hits++;
    mc_rtc::log::info("[HammeringSwing] Trajectory duration reached at lowest hammer position. Completing swing.");
    output("STOP");
    return true;
  }

  if(stop_)
  {
    mc_rtc::log::info("[HammeringSwing] Stop button clicked by user.");
    output("STOP");
    return true;
  }

  return false;
}

void HammeringSwing::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  ctl.gui()->removeElement({}, ctl.params_.stop_hammering_button_name);

  if(bspline_task_)
  {
    ctl.solver().removeTask(bspline_task_);
    bspline_task_.reset();
  }
  if(vector_orientation_task_)
  {
    ctl.solver().removeTask(vector_orientation_task_);
    vector_orientation_task_.reset();
  }
  ctl.bspline_active_ = false;
  ctl.trajectories_executed++;

  if(ctl.impulseConstraint)
  {
    ctl.solver().removeConstraintSet(*ctl.impulseConstraint);
    ctl.impulseConstraint.reset();
  }
  ctl.impulsive_constraint_flag = false;

  auto posture = ctl.getPostureTask(ctl.robot().name());
  if(posture)
  {
    posture->refAccel(Eigen::VectorXd::Zero(ctl.robot().tvmRobot().qJoints()->size()));
  }

  rm_logs(ctl_);
  mc_rtc::log::info("[HammeringSwing] Tasks and constraints cleaned up.");
}

double HammeringSwing::vector_error(const Eigen::Vector3d & va, const Eigen::Vector3d & vb) const
{
  double cos_angle = va.dot(vb) / (va.norm() * vb.norm());
  cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
  return std::acos(cos_angle);
}

Eigen::Vector3d HammeringSwing::bezier_vel_from_task(const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> & task, double dt) const
{
  const auto & bezier = task->spline().get_bezier();
  if(!bezier) return Eigen::Vector3d::Zero();
  Eigen::Vector3d p_past = (*bezier)(total_time_elapsed_ - dt);
  Eigen::Vector3d p_future = (*bezier)(total_time_elapsed_ + dt);
  return (p_future - p_past) / (2.0 * dt);
}

void HammeringSwing::add_logs(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  ctl.logger().addLogEntry("HammeringSwing_eval", this, [this]() {
    return bspline_task_ ? bspline_task_->eval().norm() : 0.0;
  });
  ctl.logger().addLogEntry("HammeringSwing_time", this, [this]() {
    return total_time_elapsed_;
  });
}

void HammeringSwing::rm_logs(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  ctl.logger().removeLogEntry("HammeringSwing_eval");
  ctl.logger().removeLogEntry("HammeringSwing_time");
}

EXPORT_SINGLE_STATE("HammeringSwing", HammeringSwing)
