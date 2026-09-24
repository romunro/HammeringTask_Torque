#include "HammeringTask_Torque.h"

void HammeringTask_Torque::add_logs()
{
  logger().addLogEntry("Hammer tip velocity controller", this, [&, this]() { return end_effector_velocity; });
  logger().addLogEntry("ImpulsiveTorquePredicted_Qptorque", this, [&, this]() { return tau_imp; });
  logger().addLogEntry("ImpulsiveTorquePredicted_Actual", this, [&, this]() { return tau_imp_act; });
  logger().addLogEntry("ImpulsiveTorquePredicted_derivative", this, [&, this]() { return tau_imp_derivate; });
  logger().addLogEntry("ImpulsiveTorquePredicted_actual_numderivative", this, [&, this]() { return tau_imp_derivate_num; });
  logger().addLogEntry("ImpulsiveTorquePredicted_low_limit_derivative", this, [&, this]() { return tau_imp_derivate_low_limit; });
  logger().addLogEntry("ImpulsiveTorquePredicted_high_limit_derivative", this, [&, this]() { return tau_imp_derivate_high_limit; });
  logger().addLogEntry("ImpulsiveTorquesimulated_speed", this, [&, this]() { return tau_imp_true_speed; });
  logger().addLogEntry("ImpulsiveTorquesimulated_force", this, [&, this]() { return tau_imp_true_force; });
  logger().addLogEntry("Effective mass [kg]", this, [&, this]() { return effective_mass; });
  logger().addLogEntry("Effective mass derivative", this, [&, this]() { return effective_mass_d; });
  logger().addLogEntry("Effective mass double derivative", this, [&, this]() { return effective_mass_dd; });
  logger().addLogEntry("Effective mass diff checker", this, [&, this]() { return eff_mass_diff_checker; });
  logger().addLogEntry("Hammer tip velocity [m/s]", this, [&, this]() { return hammer_tip_actual_velocity_vector; });
  logger().addLogEntry("Hammer tip reference bezier velocity [m/s]", this, [&, this]() { return hammer_tip_reference_velocity_vector; });
  logger().addLogEntry("Hammer tip position [m]", this, [&, this]() { return hammer_tip_actual_position_vector; });
  logger().addLogEntry("Hammer tip position real robot [m]", this, [&, this]() { return hammer_tip_actual_position_vector_realrobot; });
  logger().addLogEntry("floating base body sensor_position", this, [&, this]() { return floatingBaseSensor_.position(); });
  logger().addLogEntry("floating base body sensor_orientation", this, [&, this]() { return floatingBaseSensor_.orientation(); });
  logger().addLogEntry("floating base body sensor_linearvelocity", this, [&, this]() { return floatingBaseSensor_.linearVelocity(); });
  logger().addLogEntry("floating base body sensor_angularvelocity", this, [&, this]() { return floatingBaseSensor_.angularVelocity(); });
  logger().addLogEntry("floating base body sensor_linearacceleration", this, [&, this]() { return floatingBaseSensor_.linearAcceleration(); });
  logger().addLogEntry("floating base body sensor_angularacceleration", this, [&, this]() { return floatingBaseSensor_.angularAcceleration(); });
  logger().addLogEntry("floating base observer error position", this, [&, this]() { return floating_base_position_observer_error; });
  logger().addLogEntry("Robot left hand force sensor", this, [&, this]() { return robot().forceSensor("LeftHandForceSensor").force(); });
  logger().addLogEntry("Hammer tip observer error position", this, [&, this]() { return hammer_tip_position_observer_error; });
  logger().addLogEntry("Projected momentum of hammer tip [kgm/s]", this, [&, this]() { return projected_momentum_of_hammer_tip; });
  logger().addLogEntry("bspline_active", this, [&, this]() { return 100 * bspline_active_; });
  logger().addLogEntry("Stabilizing_speed_norm", this, [&, this]() { return stabilizing_speed_norm; });
  logger().addLogEntry("Stabilizing_eval_norm", this, [&, this]() { return stabilizing_eval_norm; });
  logger().addLogEntry("Completed_trajectories", this, [&, this]() { return trajectories_executed; });
  logger().addLogEntry("Number of hits", this, [&, this]() { return number_of_hits; });

  logger().addLogEntry("posture_task_weight", this, [this]() {
    auto p = getPostureTask(robot().name());
    return p ? p->weight() : 0.0;
  });
  logger().addLogEntry("posture_task_stiffness", this, [this]() {
    auto p = getPostureTask(robot().name());
    return p ? p->stiffness() : 0.0;
  });

  for(const auto & jname : mass_maximization_active_joints)
  {
    if(robot().hasJoint(jname))
    {
      auto jIndex = robot().jointIndexByName(jname);
      int dofIndex = static_cast<int>(robot().mb().jointPosInDof(static_cast<int>(jIndex)) - robot().mb().joint(0).dof());
      logger().addLogEntry("posture_weight_" + jname, this, [this, dofIndex]() {
        auto p = getPostureTask(robot().name());
        if(!p) return 0.0;
        const auto & dw = p->dimWeight();
        if(dofIndex >= 0 && dofIndex < dw.size())
        {
          return p->weight() * dw(dofIndex);
        }
        return p->weight();
      });
      logger().addLogEntry("posture_stiffness_" + jname, this, [this, dofIndex]() {
        auto p = getPostureTask(robot().name());
        if(!p) return 0.0;
        const auto & dw = p->dimWeight();
        if(dofIndex >= 0 && dofIndex < dw.size() && dw(dofIndex) <= 1e-6)
        {
          return 0.0;
        }
        return p->stiffness();
      });
      logger().addLogEntry("posture_dimWeight_" + jname, this, [this, dofIndex]() {
        auto p = getPostureTask(robot().name());
        if(!p) return 0.0;
        const auto & dw = p->dimWeight();
        if(dofIndex >= 0 && dofIndex < dw.size())
        {
          return dw(dofIndex);
        }
        return 1.0;
      });
    }
  }

  logger().addLogEntry("com_eval_norm", this, [&, this]() { return com_eval_norm; });
  logger().addLogEntry("pelvis_eval_norm", this, [&, this]() { return pelvis_eval_norm; });
  logger().addLogEntry("torso_eval_norm", this, [&, this]() { return torso_eval_norm; });
  logger().addLogEntry("contacts_eval_norm", this, [&, this]() { return contacts_eval_norm; });
}
