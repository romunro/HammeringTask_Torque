#include "HammeringSwing.h"
#include "../HammeringTask_Torque.h"
#include <mc_rtc/logging.h>
#include <mc_rbdyn/PlanarSurface.h>
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

  // ------------------------- Effective mass maximization task update ----------------------------
  // See Jimmy paper to understand why we use posture task

  int number_of_joints = ctl.robot().tvmRobot().qJoints()->size();

  Eigen::MatrixXd joint_selector = Eigen::MatrixXd::Zero(number_of_joints, number_of_joints);
  for(const auto & joint: mass_maximization_active_joints)
  {
    auto joint_index = jointIndex(joint);
    if(joint_index >= 0 && joint_index < number_of_joints)
    {
      joint_selector(joint_index, joint_index) = 1.0;
    }
  }

  _new_mbc = ctl.robot().mbc();
  _gradient_of_m = compute_emass_gradient_backward_difference_mbc(_new_mbc, ctl, ctl.nail_normal_vector_world_frame, mass_maximization_active_joints);

  double arm_weight = ctl.params_.posture.arm_nullspace_weight > 1e-6 ? ctl.params_.posture.arm_nullspace_weight : 0.01;
  Eigen::VectorXd feedforward_term = (ctl.params_.trajectory.effective_mass_maximization_weight / arm_weight) * joint_selector * _gradient_of_m.tail(number_of_joints);
  ctl.getPostureTask(ctl.robot().name())->refAccel(feedforward_term);
  
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

const double HammeringSwing::compute_effective_mass_with_mbc(
    rbd::MultiBodyConfig mbc,
    mc_control::fsm::Controller &ctl_,
    const Eigen::Vector3d &normal_vector) const
{
    HammeringTask_Torque &ctl = static_cast<HammeringTask_Torque &>(ctl_);
    ctl.robot().forwardKinematics(mbc);
    const rbd::MultiBody &robot_mb = ctl.robot().mb();
    rbd::Jacobian jac(robot_mb, ctl.params_.hammer_head_frame);
    const Eigen::MatrixXd world_jacobian = jac.jacobian(robot_mb, mbc);
    Eigen::MatrixXd full_jacobian(6, robot_mb.nrDof());
    jac.fullJacobian(robot_mb, world_jacobian, full_jacobian);
    const Eigen::MatrixXd Jv = full_jacobian.bottomRows(3);
    
    rbd::ForwardDynamics fd(robot_mb);
    fd.computeH(robot_mb, mbc);
    const Eigen::MatrixXd &M = fd.H();
    auto M_ldlt = M.ldlt();

    const Eigen::VectorXd JvT_n = Jv.transpose() * normal_vector;
    const Eigen::VectorXd Minv_JvT_n = M_ldlt.solve(JvT_n);
    
    double unconstrained_inv_inertia = JvT_n.dot(Minv_JvT_n);
    
    // Compute contact jacobians for active contacts dynamically
    struct ContactInfo {
        std::string bodyName;
        double mu;
        double X_foot;
        double Y_foot;
    };
    std::vector<ContactInfo> active_contacts;
    
    for(const auto & contact : ctl.contacts())
    {
        std::string r1_name = contact.r1.value_or(ctl.robot().name());
        std::string r2_name = contact.r2.value_or(ctl.robot().name());
        
        std::string surface_name = "";
        if(r1_name == ctl.robot().name()) {
            surface_name = contact.r1Surface;
        } else if(r2_name == ctl.robot().name()) {
            surface_name = contact.r2Surface;
        }
        
        if(!surface_name.empty() && ctl.robot().hasSurface(surface_name))
        {
            const mc_rbdyn::Surface & surf = ctl.robot().surface(surface_name);
            ContactInfo info;
            info.bodyName = surf.bodyName();
            info.mu = contact.friction;
            info.X_foot = 0.05; // Fallback defaults
            info.Y_foot = 0.05;
            
            const mc_rbdyn::PlanarSurface * ps = dynamic_cast<const mc_rbdyn::PlanarSurface*>(&surf);
            if(ps)
            {
                const auto & pts = ps->planarPoints();
                if(!pts.empty())
                {
                    double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
                    for(const auto & pt : pts)
                    {
                        min_x = std::min(min_x, pt.first);
                        max_x = std::max(max_x, pt.first);
                        min_y = std::min(min_y, pt.second);
                        max_y = std::max(max_y, pt.second);
                    }
                    info.X_foot = std::max(std::abs(min_x), std::abs(max_x));
                    info.Y_foot = std::max(std::abs(min_y), std::abs(max_y));
                }
            }
            active_contacts.push_back(info);
        }
    }

    Eigen::MatrixXd Jc(6 * active_contacts.size(), robot_mb.nrDof());
    Jc.setZero();
    for(size_t i = 0; i < active_contacts.size(); ++i)
    {
        if(ctl.robot().hasBody(active_contacts[i].bodyName))
        {
            rbd::Jacobian jac_c(robot_mb, active_contacts[i].bodyName);
            Eigen::MatrixXd Jc_world = jac_c.jacobian(robot_mb, mbc);
            Eigen::MatrixXd Jc_full(6, robot_mb.nrDof());
            jac_c.fullJacobian(robot_mb, Jc_world, Jc_full);
            Jc.middleRows(i * 6, 6) = Jc_full;
        }
    }
    
    Eigen::MatrixXd Minv_JcT = Eigen::MatrixXd::Zero(robot_mb.nrDof(), Jc.rows());
    for(int i = 0; i < Jc.rows(); ++i)
    {
        Minv_JcT.col(i) = M_ldlt.solve(Jc.row(i).transpose());
    }
    Eigen::MatrixXd Lambda_contact = Jc * Minv_JcT;
    Lambda_contact.diagonal().array() += 1e-5; // Damping for singularities/overconstraints
    
    Eigen::VectorXd v2 = Jc * Minv_JvT_n;
    
    // Required impulsive contact reaction if perfectly bolted
    Eigen::VectorXd p_bolted = -Lambda_contact.ldlt().solve(v2);
    
    // Project the bolted impulse onto the exact URDF Unilateral and Friction Cone limits
    Eigen::VectorXd p_actual = p_bolted;
    double mu_tau = 0.05; // Constant torsional friction
    
    for(size_t i = 0; i < active_contacts.size(); ++i)
    {
        int idx = i * 6;
        double f_z = p_actual(idx + 5);
        
        if(f_z <= 0.0)
        {
            p_actual.segment<6>(idx).setZero();
        }
        else
        {
            double f_lat = std::sqrt(p_actual(idx + 3)*p_actual(idx + 3) + p_actual(idx + 4)*p_actual(idx + 4));
            double mu = active_contacts[i].mu;
            if(f_lat > mu * f_z)
            {
                p_actual(idx + 3) *= (mu * f_z) / f_lat;
                p_actual(idx + 4) *= (mu * f_z) / f_lat;
            }
            
            double X_limit = active_contacts[i].X_foot * f_z;
            double Y_limit = active_contacts[i].Y_foot * f_z;
            p_actual(idx + 0) = std::max(-Y_limit, std::min(p_actual(idx + 0), Y_limit));
            p_actual(idx + 1) = std::max(-X_limit, std::min(p_actual(idx + 1), X_limit));
            p_actual(idx + 2) = std::max(-mu_tau * f_z, std::min(p_actual(idx + 2), mu_tau * f_z));
        }
    }
    
    // Actual unilateral/frictional constrained inverse inertia:
    // Delta_dot_x = J_v M^-1 J_v^T n + J_v M^-1 J_c^T p_actual
    // n^T Delta_dot_x = unconstrained_inv_inertia + v2^T p_actual
    double actual_inv_inertia = unconstrained_inv_inertia + v2.dot(p_actual);
    
    if(std::abs(actual_inv_inertia) < 1e-12)
    {
        return 0.0;
    }
    return 1.0 / actual_inv_inertia;
}

Eigen::VectorXd HammeringSwing::compute_emass_gradient_backward_difference_mbc(
    const rbd::MultiBodyConfig &mbc,
    mc_control::fsm::Controller &ctl_,
    const Eigen::Vector3d &normal_vector,
    const std::vector<std::string> &active_joints) const
{
    HammeringTask_Torque &ctl = static_cast<HammeringTask_Torque &>(ctl_);
    const rbd::MultiBody &robot_mb = ctl.robot().mb();
    constexpr double epsilon = 1e-5;
    
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(robot_mb.nrDof());
    const double effective_mass_current = compute_effective_mass_with_mbc(mbc, ctl_, normal_vector);

    for(const auto &joint_name : active_joints)
    {
        const unsigned int joint_mb_index = robot_mb.jointIndexByName(joint_name);
        const int dof_index = robot_mb.jointPosInDof(joint_mb_index);

        rbd::MultiBodyConfig mbc_minus = mbc;
        mbc_minus.q[joint_mb_index][0] -= epsilon;

        const double effective_mass_minus = compute_effective_mass_with_mbc(mbc_minus, ctl_, normal_vector);

        gradient(dof_index) = (effective_mass_current - effective_mass_minus) / epsilon;
    }
    return gradient;
}

EXPORT_SINGLE_STATE("HammeringSwing", HammeringSwing)
