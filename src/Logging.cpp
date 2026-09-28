#include "HammeringTask_Torque.h"
#include <fmt/format.h>

void HammeringTask_Torque::add_logs()
{
  logger().addLogEntry("Hammer tip velocity controller", this, [this]() { return end_effector_velocity; });
  logger().addLogEntry("ImpulsiveTorquePredicted_Qptorque", this, [this]() { return tau_imp; });
  logger().addLogEntry("ImpulsiveTorquePredicted_Actual", this, [this]() { return tau_imp_act; });
  logger().addLogEntry("ImpulsiveTorquePredicted_derivative", this, [this]() { return tau_imp_derivate; });
  logger().addLogEntry("ImpulsiveTorquePredicted_derivative_qp", this, [this]() { return tau_imp_derivate_qp; });
  logger().addLogEntry("ImpulsiveTorquePredicted_derivative_actual", this, [this]() { return tau_imp_derivate_act; });
  logger().addLogEntry("ImpulsiveTorquePredicted_actual_numderivative", this, [this]() { return tau_imp_derivate_num; });
  logger().addLogEntry("ImpulsiveTorquePredicted_numderivative", this, [this]() { return tau_imp_derivate_num; });
  logger().addLogEntry("ImpulsiveTorquePredicted_low_limit_derivative", this, [this]() { return tau_imp_derivate_low_limit; });
  logger().addLogEntry("ImpulsiveTorquePredicted_high_limit_derivative", this, [this]() { return tau_imp_derivate_high_limit; });
  logger().addLogEntry("ImpulsiveTorquesimulated_speed", this, [this]() { return tau_imp_true_speed; });
  logger().addLogEntry("ImpulsiveTorquesimulated_force", this, [this]() { return tau_imp_true_force; });
  logger().addLogEntry("Effective mass [kg]", this, [this]() { return effective_mass; });
  logger().addLogEntry("Effective mass derivative", this, [this]() { return effective_mass_d; });
  logger().addLogEntry("Effective mass double derivative", this, [this]() { return effective_mass_dd; });
  logger().addLogEntry("Effective mass diff checker", this, [this]() { return eff_mass_diff_checker; });
  logger().addLogEntry("Hammer tip velocity [m/s]", this, [this]() { return hammer_tip_actual_velocity_vector; });
  logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [this]() { return hammer_tip_reference_velocity_vector; });
  logger().addLogEntry("Hammer tip position [m]", this, [this]() { return hammer_tip_actual_position_vector; });
  logger().addLogEntry("Hammer tip position real robot [m]", this, [this]() { return hammer_tip_actual_position_vector_realrobot; });
  logger().addLogEntry("floating base body sensor_position", this, [this]() { return floatingBaseSensor_.position(); });
  logger().addLogEntry("floating base body sensor_orientation", this, [this]() { return floatingBaseSensor_.orientation(); });
  logger().addLogEntry("floating base body sensor_linearvelocity", this, [this]() { return floatingBaseSensor_.linearVelocity(); });
  logger().addLogEntry("floating base body sensor_angularvelocity", this, [this]() { return floatingBaseSensor_.angularVelocity(); });
  logger().addLogEntry("floating base body sensor_linearacceleration", this, [this]() { return floatingBaseSensor_.linearAcceleration(); });
  logger().addLogEntry("floating base body sensor_angularacceleration", this, [this]() { return floatingBaseSensor_.angularAcceleration(); });
  logger().addLogEntry("floating base observer error position", this, [this]() { return floating_base_position_observer_error; });
  logger().addLogEntry("Robot left hand force sensor", this, [this]() { return robot().forceSensor("LeftHandForceSensor").force(); });
  logger().addLogEntry("Hammer tip observer error position", this, [this]() { return hammer_tip_position_observer_error; });
  logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [this]() { return projected_momentum_of_hammer_tip; });
  logger().addLogEntry("bspline_active", this, [this]() { return 100 * bspline_active_; });
  logger().addLogEntry("Stabilizing_speed_norm", this, [this]() { return stabilizing_speed_norm; });
  logger().addLogEntry("Stabilizing_eval_norm", this, [this]() { return stabilizing_eval_norm; });
  logger().addLogEntry("Completed_trajectories", this, [this]() { return trajectories_executed; });
  logger().addLogEntry("Number of hits", this, [this]() { return number_of_hits; });

  logger().addLogEntry("posture_task_weight", this, [this]() {
    auto p = getPostureTask(robot().name());
    return p ? p->weight() : 0.0;
  });
  logger().addLogEntry("posture_task_stiffness", this, [this]() {
    auto p = getPostureTask(robot().name());
    return p ? p->stiffness() : 0.0;
  });

  // -------------------------------------------------------------------------
  // Grouped Constraints & Tasks logging:
  // DOF Indexing and Floating Base Considerations:
  // - MultiBody joint 0 is the 6-DOF floating base (DOFs 0..5).
  // - Actuated joints begin at MultiBody DOF index 6 (posInDof >= 6).
  // - Vectors sized robot().mb().nrDof() (e.g. tau_imp, tau_imp_act,
  //   tau_imp_derivate, limits().tl, limits().tu, qdm, etc.) include the 6 DOFs
  //   of the floating base at indices 0..5, so actuated joints are at index 'dof'.
  // - PostureTask dimWeight has dimension (nrDof - 6), so its index is dof - 6.
  // - robot().mbc().q, robot().mbc().alpha, robot().mbc().jointTorque, and
  //   robot().ql(), qu(), tl(), tu() are indexed by MultiBody joint index 'jIndex'.
  // -------------------------------------------------------------------------
  int floatingBaseDofs = robot().mb().nrJoints() > 0 ? robot().mb().joint(0).dof() : 0;

  for(size_t jIndex = 0; jIndex < robot().mb().nrJoints(); ++jIndex)
  {
    const auto & joint = robot().mb().joint(static_cast<int>(jIndex));
    if(joint.type() == rbd::Joint::Free) continue; // Skip floating base
    if(joint.dof() == 0) continue;                 // Skip fixed joints

    const auto & jname = joint.name();
    int dof = static_cast<int>(robot().mb().jointPosInDof(static_cast<int>(jIndex)));
    int dofIndex_posture = dof - floatingBaseDofs;

    // 1. Impulsive Torque Derivative Constraint Set (actual, qp, num, lower, upper)
    std::string p_imp_deriv = fmt::format("Constraints_ImpulsiveTorqueDerivative_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_imp_deriv + "_actual", this, [this, dof]() {
      return (tau_imp_derivate_act.size() > dof) ? tau_imp_derivate_act(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_deriv + "_qp", this, [this, dof]() {
      return (tau_imp_derivate_qp.size() > dof) ? tau_imp_derivate_qp(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_deriv + "_num", this, [this, dof]() {
      return (tau_imp_derivate_num.size() > dof) ? tau_imp_derivate_num(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_deriv + "_lower", this, [this, dof]() {
      return (tau_imp_derivate_low_limit.size() > dof) ? tau_imp_derivate_low_limit(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_deriv + "_upper", this, [this, dof]() {
      return (tau_imp_derivate_high_limit.size() > dof) ? tau_imp_derivate_high_limit(dof) : 0.0;
    });

    // 2. Impulsive Torque Constraint Set (actual, QP command, lower limit, upper limit, simulated speed/force)
    std::string p_imp_tau = fmt::format("Constraints_ImpulsiveTorque_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_imp_tau + "_actual", this, [this, dof]() {
      return (tau_imp_act.size() > dof) ? tau_imp_act(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_tau + "_qp", this, [this, dof]() {
      return (tau_imp.size() > dof) ? tau_imp(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_tau + "_lower", this, [this, dof]() {
      if(impulseConstraint && impulsive_constraint_flag && impulseConstraint->EffectiveLambda().size() > dof)
      {
        return impulseConstraint->TorqueLowerLimit()(dof);
      }
      return (robot().tvmRobot().limits().tl.size() > dof) ? (robot().tvmRobot().limits().tl(dof) * params_.impulse.limit_multiplier) : 0.0;
    });
    logger().addLogEntry(p_imp_tau + "_upper", this, [this, dof]() {
      if(impulseConstraint && impulsive_constraint_flag && impulseConstraint->EffectiveLambda().size() > dof)
      {
        return impulseConstraint->TorqueHigherLimit()(dof);
      }
      return (robot().tvmRobot().limits().tu.size() > dof) ? (robot().tvmRobot().limits().tu(dof) * params_.impulse.limit_multiplier) : 0.0;
    });
    logger().addLogEntry(p_imp_tau + "_sim_speed", this, [this, dof]() {
      return (tau_imp_true_speed.size() > dof) ? tau_imp_true_speed(dof) : 0.0;
    });
    logger().addLogEntry(p_imp_tau + "_sim_force", this, [this, dof]() {
      return (tau_imp_true_force.size() > dof) ? tau_imp_true_force(dof) : 0.0;
    });

    // 3. Robot Joint Torque Limits Constraint Set (actual, lower, upper)
    std::string p_joint_tau = fmt::format("Constraints_JointTorque_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_joint_tau + "_actual", this, [this, jIndex]() {
      const auto & jt = robot().mbc().jointTorque;
      return (jt.size() > jIndex && !jt[jIndex].empty()) ? jt[jIndex][0] : 0.0;
    });
    logger().addLogEntry(p_joint_tau + "_lower", this, [this, dof]() {
      return (robot().tvmRobot().limits().tl.size() > dof) ? robot().tvmRobot().limits().tl(dof) : 0.0;
    });
    logger().addLogEntry(p_joint_tau + "_upper", this, [this, dof]() {
      return (robot().tvmRobot().limits().tu.size() > dof) ? robot().tvmRobot().limits().tu(dof) : 0.0;
    });

    // 4. Robot Joint Velocity Limits Constraint Set (actual, command, lower, upper)
    std::string p_joint_vel = fmt::format("Constraints_JointVelocity_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_joint_vel + "_actual", this, [this, dof]() {
      return (qdm.size() > dof) ? qdm(dof) : 0.0;
    });
    logger().addLogEntry(p_joint_vel + "_command", this, [this, dof]() {
      const auto & alpha = robot().tvmRobot().alpha()->value();
      return (alpha.size() > dof) ? alpha(dof) : 0.0;
    });
    logger().addLogEntry(p_joint_vel + "_lower", this, [this, dof]() {
      return (robot().tvmRobot().limits().vl.size() > dof) ? robot().tvmRobot().limits().vl(dof) : 0.0;
    });
    logger().addLogEntry(p_joint_vel + "_upper", this, [this, dof]() {
      return (robot().tvmRobot().limits().vu.size() > dof) ? robot().tvmRobot().limits().vu(dof) : 0.0;
    });

    // 5. Robot Joint Position Limits Constraint Set (actual, lower, upper)
    std::string p_joint_pos = fmt::format("Constraints_JointPosition_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_joint_pos + "_actual", this, [this, jIndex]() {
      const auto & q = robot().mbc().q;
      return (q.size() > jIndex && !q[jIndex].empty()) ? q[jIndex][0] : 0.0;
    });
    logger().addLogEntry(p_joint_pos + "_lower", this, [this, jIndex]() {
      const auto & ql = robot().ql();
      return (ql.size() > jIndex && !ql[jIndex].empty()) ? ql[jIndex][0] : 0.0;
    });
    logger().addLogEntry(p_joint_pos + "_upper", this, [this, jIndex]() {
      const auto & qu = robot().qu();
      return (qu.size() > jIndex && !qu[jIndex].empty()) ? qu[jIndex][0] : 0.0;
    });

    // 6. Posture Task per joint Set (weight, stiffness, dimWeight)
    std::string p_posture = fmt::format("Tasks_Posture_{:02d}_{}", dof, jname);
    logger().addLogEntry(p_posture + "_weight", this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size())
      {
        return p->weight() * dw(dofIndex_posture);
      }
      return p->weight();
    });
    logger().addLogEntry(p_posture + "_stiffness", this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size() && dw(dofIndex_posture) <= 1e-6)
      {
        return 0.0;
      }
      return p->stiffness();
    });
    logger().addLogEntry(p_posture + "_dimWeight", this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size())
      {
        return dw(dofIndex_posture);
      }
      return 1.0;
    });

    // Legacy posture entries for backwards compatibility
    logger().addLogEntry("posture_weight_" + jname, this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size())
      {
        return p->weight() * dw(dofIndex_posture);
      }
      return p->weight();
    });
    logger().addLogEntry("posture_stiffness_" + jname, this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size() && dw(dofIndex_posture) <= 1e-6)
      {
        return 0.0;
      }
      return p->stiffness();
    });
    logger().addLogEntry("posture_dimWeight_" + jname, this, [this, dofIndex_posture]() {
      auto p = getPostureTask(robot().name());
      if(!p) return 0.0;
      const auto & dw = p->dimWeight();
      if(dofIndex_posture >= 0 && dofIndex_posture < dw.size())
      {
        return dw(dofIndex_posture);
      }
      return 1.0;
    });
  }

  logger().addLogEntry("com_eval_norm", this, [&, this]() { return com_eval_norm; });
  logger().addLogEntry("pelvis_eval_norm", this, [&, this]() { return pelvis_eval_norm; });
  logger().addLogEntry("torso_eval_norm", this, [&, this]() { return torso_eval_norm; });
  logger().addLogEntry("contacts_eval_norm", this, [&, this]() { return contacts_eval_norm; });
}

