#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/ContactConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include "api.h"

struct HammeringTask_Torque_DLLAPI HammeringTask_Torque : public mc_control::fsm::Controller
{
  HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void load_parameters();
  void apply_parameters();
  void addToGUI();

  // Constraints
  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintSet;
  std::unique_ptr<mc_solver::DynamicsConstraint> dynamicsConstraint;
  std::array<double, 3> _damping = {0.1, 0.01, 0.5};
  double _vp = 0.9;
  bool _infTorque = false;

  // LIPM Stabilizer Task
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf;

  // Stabilizer Parameters (Stage 1: Minimum to Stand)
  double _torso_task_stiffness = 10.0;
  double _torso_task_weight = 100.0;
  double _torso_pitch = 0.0;
  double _pelvis_task_stiffness = 100.0;
  double _pelvis_task_weight = 1000.0;
  Eigen::Vector2d _dcm_p = Eigen::Vector2d::Zero();
  Eigen::Vector2d _dcm_i = Eigen::Vector2d::Zero();
  Eigen::Vector2d _dcm_d = Eigen::Vector2d::Zero();
  Eigen::Vector3d _com_stiffness = {100.0, 100.0, 500.0};
  double _com_weight = 1000.0;
  Eigen::Vector3d _com_dim_weight = Eigen::Vector3d::Ones();
  double _com_height = 0.84;
  bool _has_com_height = false;
  double _contact_task_weight = 100000.0;
  sva::MotionVecd _contact_stiffness = sva::MotionVecd({1.0, 1.0, 1.0}, {1.0, 1.0, 1.0});
  sva::MotionVecd _contact_damping = sva::MotionVecd({300.0, 300.0, 300.0}, {300.0, 300.0, 300.0});
  Eigen::Vector2d _contact_admittance = Eigen::Vector2d::Zero();
  mc_rbdyn::Gains3d _df_admittance = {0.0, 0.0, 0.0};

  // Posture Task Parameters (Stage 1)
  double base_posture_stiffness = 10.0;
  double base_posture_weight = 1.0;

  // Control mode in datastore (e.g. "Torque" or "Position")
  std::string control_mode = "Torque";

private:
  mc_rtc::Configuration config_;
};