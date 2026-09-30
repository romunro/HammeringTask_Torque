#include "HammeringTask_Torque.h"

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ComboInput.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/plot.h>

void HammeringTask_Torque::addToGUI()
{
  // 1. Trajectory configuration
  this->gui()->addElement({"Hammering task", "Trajectory"},
    mc_rtc::gui::NumberInput("BSpline Duration [s]",
      [this]() { return params_.trajectory.duration; },
      [this](double d) { params_.trajectory.duration = d; }),
    mc_rtc::gui::ArrayInput("Final Velocity [x, y, z] (m/s)",
      [this]() { return params_.trajectory.final_velocity; },
      [this](const Eigen::Vector3d & v) { params_.trajectory.final_velocity = v; }),
    mc_rtc::gui::ArrayInput("Initial Velocity [x, y, z] (m/s)",
      [this]() { return params_.trajectory.init_velocity; },
      [this](const Eigen::Vector3d & v) { params_.trajectory.init_velocity = v; }),
    mc_rtc::gui::ArrayInput("Hitting Target [x, y, z] (m)",
      [this]() { return params_.trajectory.nail_target_position; },
      [this](const Eigen::Vector3d & target) { params_.trajectory.nail_target_position = target; }),
    mc_rtc::gui::NumberInput("Waypoint Height [m]",
      [this]() { return params_.trajectory.waypoint_height; },
      [this](double h) { params_.trajectory.waypoint_height = h; }),
    mc_rtc::gui::NumberInput("Task Stiffness",
      [this]() { return params_.trajectory.stiffness; },
      [this](double k) { params_.trajectory.stiffness = k; }),
    mc_rtc::gui::NumberInput("Task Damping",
      [this]() { return params_.trajectory.damping; },
      [this](double d) { params_.trajectory.damping = d; }),
    mc_rtc::gui::NumberInput("Task Weight",
      [this]() { return params_.trajectory.weight; },
      [this](double w) { params_.trajectory.weight = w; }),
    mc_rtc::gui::ArrayInput("Dimension Weights [rx, ry, rz, tx, ty, tz]",
      [this]() { return params_.trajectory.dimweights; },
      [this](const Eigen::Vector6d & dw) { params_.trajectory.dimweights = dw; })
  );

  // 2. Impulse Constraint configuration
  this->gui()->addElement({"Hammering task", "Impulse Constraint"},
    mc_rtc::gui::NumberInput("Restitution (c_res)",
      [this]() { return params_.impulse.c_res; },
      [this](double val) { params_.impulse.c_res = val; }),
    mc_rtc::gui::NumberInput("Time window dt [s]",
      [this]() { return params_.impulse.delta_t; },
      [this](double val) { params_.impulse.delta_t = val; }),
    mc_rtc::gui::NumberInput("Torque Limit Multiplier",
      [this]() { return params_.impulse.limit_multiplier; },
      [this](double val) { params_.impulse.limit_multiplier = val; }),
    mc_rtc::gui::NumberInput("Tau High Multiplier",
      [this]() { return params_.impulse.tau_high_multiplier; },
      [this](double val) { params_.impulse.tau_high_multiplier = val; }),
    mc_rtc::gui::NumberInput("Lambda High",
      [this]() { return params_.impulse.lambda_high; },
      [this](double val) { params_.impulse.lambda_high = val; }),
    mc_rtc::gui::NumberInput("Lambda Low",
      [this]() { return params_.impulse.lambda_low; },
      [this](double val) { params_.impulse.lambda_low = val; }),
    mc_rtc::gui::NumberInput("Distance Ratio (K)",
      [this]() { return params_.impulse.K; },
      [this](double val) { params_.impulse.K = val; }),
    mc_rtc::gui::NumberInput("Activation Height [m]",
      [this]() { return params_.impulse.activation_height; },
      [this](double val) { params_.impulse.activation_height = val; }),
    mc_rtc::gui::NumberInput("Impact Force Threshold [N]",
      [this]() { return params_.impact.sensor_force_threshold; },
      [this](double val) { params_.impact.sensor_force_threshold = val; })
  );

  // 3. Controller Status & Controls
  this->gui()->addElement({"Hammering task", "Status"},
    mc_rtc::gui::Checkbox(params_.linear_constraint_button_name,
      [this]() { return params_.impulse.linear_impulsive_torque_ctr_flag; },
      [this]() { params_.impulse.linear_impulsive_torque_ctr_flag = !params_.impulse.linear_impulsive_torque_ctr_flag; }),
    mc_rtc::gui::Label("Impact detected", [this]() -> bool { return impact_detected; }),
    mc_rtc::gui::Label("Impulse constraint active", [this]() -> bool { return impulsive_constraint_flag; }),
    mc_rtc::gui::Label("Completed trajectories", [this]() { return trajectories_executed; }),
    mc_rtc::gui::Label("Number of hits", [this]() { return number_of_hits; })
  );

  using Color = mc_rtc::gui::Color;
  using Style = mc_rtc::gui::plot::Style;
  using AxisConfig = mc_rtc::gui::plot::AxisConfiguration;

  auto get_plot_upper = [this](int dof) -> double {
    if(selected_plot_mode_ == "Derivative of Impulsive Torque")
    {
      return (tau_imp_derivate_high_limit.size() > dof) ? tau_imp_derivate_high_limit(dof) : 0.0;
    }
    if(impulseConstraint && impulseConstraint->TorqueHigherLimit().size() > dof)
    {
      return impulseConstraint->TorqueHigherLimit()(dof);
    }
    return robot().tvmRobot().limits().tu(dof) * params_.impulse.limit_multiplier;
  };

  auto get_plot_lower = [this](int dof) -> double {
    if(selected_plot_mode_ == "Derivative of Impulsive Torque")
    {
      return (tau_imp_derivate_low_limit.size() > dof) ? tau_imp_derivate_low_limit(dof) : 0.0;
    }
    if(impulseConstraint && impulseConstraint->TorqueLowerLimit().size() > dof)
    {
      return impulseConstraint->TorqueLowerLimit()(dof);
    }
    return robot().tvmRobot().limits().tl(dof) * params_.impulse.limit_multiplier;
  };

  auto get_plot_out = [this](int dof) -> double {
    if(selected_plot_mode_ == "Derivative of Impulsive Torque")
    {
      return (tau_imp_derivate.size() > dof) ? tau_imp_derivate(dof) : 0.0;
    }
    return (tau_imp_act.size() > dof) ? tau_imp_act(dof) : 0.0;
  };

  this->gui()->addElement({"Hammering task", "Friction Compensation"},
    mc_rtc::gui::Checkbox("Enable Friction Compensation",
      [this]() { return enable_friction_compensation_; },
      [this]() { enable_friction_compensation_ = !enable_friction_compensation_; }),
    mc_rtc::gui::NumberInput("Coulomb Friction tau_c [Nm]",
      [this]() { return params_.friction.tau_c; },
      [this](double v) { params_.friction.tau_c = std::max(0.0, v); }),
    mc_rtc::gui::NumberInput("Viscous Friction f_v [Nm/(rad/s)]",
      [this]() { return params_.friction.f_v; },
      [this](double v) { params_.friction.f_v = std::max(0.0, v); }),
    mc_rtc::gui::NumberInput("Velocity Threshold v_th [rad/s]",
      [this]() { return params_.friction.v_th; },
      [this](double v) { params_.friction.v_th = std::max(1e-4, v); }),
    mc_rtc::gui::Checkbox("Use Reference Velocity (Feedforward)",
      [this]() { return params_.friction.use_ref_vel; },
      [this]() { params_.friction.use_ref_vel = !params_.friction.use_ref_vel; })
  );

  // 5. Mass Maximization configuration & telemetry
  this->gui()->addElement({"Hammering task", "Mass Maximization"},
    mc_rtc::gui::NumberInput("Effective Mass Maximization Weight",
      [this]() { return params_.trajectory.effective_mass_maximization_weight; },
      [this](double w) { params_.trajectory.effective_mass_maximization_weight = std::max(0.0, w); }),
    mc_rtc::gui::NumberInput("Arm Nullspace Weight",
      [this]() { return params_.posture.arm_nullspace_weight; },
      [this](double w) { params_.posture.arm_nullspace_weight = std::max(1e-4, w); }),
    mc_rtc::gui::Label("Effective Mass [kg]",
      [this]() { return effective_mass; }),
    mc_rtc::gui::Label("Effective Mass Derivative [kg/s]",
      [this]() { return effective_mass_d; }),
    mc_rtc::gui::Label("Projected Momentum [kg.m/s]",
      [this]() { return projected_momentum_of_hammer_tip; })
  );

  this->gui()->addElement({"Plots"},
    mc_rtc::gui::ComboInput("Joint Selection",
      mass_maximization_active_joints,
      [this]() { return selected_plot_joint_; },
      [this](const std::string & j) { selected_plot_joint_ = j; }),
    mc_rtc::gui::ComboInput("Plot Mode",
      plot_modes_,
      [this]() { return selected_plot_mode_; },
      [this](const std::string & m) { selected_plot_mode_ = m; })
  );

  this->gui()->addPlot("Impulsive Torque Monitor",
    mc_rtc::gui::plot::X("t [s]", [this]() { return total_time_elapsed; }),
    mc_rtc::gui::plot::AxisConfiguration{"Torque [N.m] / Deriv [N.m/s]"},
    mc_rtc::gui::plot::Y("Upper Limit", [this, get_plot_upper]() { return get_plot_upper(this->get_dof(selected_plot_joint_)); }, Color::Red, Style::Dotted),
    mc_rtc::gui::plot::Y("Predicted Impulsive Torque", [this, get_plot_out]() { return get_plot_out(this->get_dof(selected_plot_joint_)); }, Color::Blue, Style::Solid),
    mc_rtc::gui::plot::Y("Lower Limit", [this, get_plot_lower]() { return get_plot_lower(this->get_dof(selected_plot_joint_)); }, Color::Red, Style::Dotted)
  );

  this->gui()->addPlot("Effective Mass Monitor",
    mc_rtc::gui::plot::X("t [s]", [this]() { return total_time_elapsed; }),
    mc_rtc::gui::plot::AxisConfiguration{"Effective Mass [kg]"},
    mc_rtc::gui::plot::Y("Effective Mass", [this]() { return effective_mass; }, Color::Blue, Style::Solid),
    mc_rtc::gui::plot::Y("Projected Momentum", [this]() { return projected_momentum_of_hammer_tip; }, Color::Green, Style::Solid)
  );
}
