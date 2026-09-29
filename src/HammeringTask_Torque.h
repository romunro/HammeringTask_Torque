#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/gui/plot.h>

#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/ContactConstraint.h>
#include <mc_solver/ImpulseConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <ndcurves/curve_constraint.h>

#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>
#include <mc_filter/LowPass.h>

#include <vector>
#include <string>
#include <memory>
#include "api.h"
#include "Parameters.h"

typedef Eigen::Vector3d Point;
typedef Point point_t;
typedef ndcurves::curve_constraints<point_t> curve_constraints_t;

#include <Tasks/QPTasks.h>

struct HammeringTask_Torque_DLLAPI HammeringTask_Torque : public mc_control::fsm::Controller
{
public:
  HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  void load_parameters();
  void apply_parameters();
  void add_logs();
  void addToGUI();

  int get_dof(const std::string & jname) const;
  double compute_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                         mc_control::fsm::Controller & ctl_, 
                                         const Eigen::Vector3d & normal_vector);
  double compute_effective_mass_d_with_mbc(rbd::MultiBodyConfig mbc, 
                                           mc_control::fsm::Controller & ctl_, 
                                           const Eigen::Vector3d & normal_vector,
                                           double effective_mass);

  // Configuration parameters (leading, loaded from YAML)
  ControllerParams params_;
  mc_rtc::Configuration config_;

  // Frame alignments
  const Eigen::Vector3d normal_vector_to_align_in_hammerhead_frame = {1, 0, 0};
  const Eigen::Vector3d normal_vector_nail_frame = {0, 0, 1};
  Eigen::Matrix3d nail_rot = Eigen::Matrix3d::Identity();
  Eigen::Vector3d nail_normal_vector_world_frame = {0, 0, 0};
  Eigen::Matrix6d P_n = Eigen::Matrix6d::Zero();

  // Runtime State
  bool impact_detected = false;
  bool impulsive_constraint_flag = false;
  bool force_felt = false;
  bool bspline_active_ = false;

  int trajectories_executed = 0;
  int number_of_hits = 0;
  double total_time_elapsed = 0.0;

  // Telemetry & Logs
  double effective_mass = 0.0;
  double effective_mass_d = 0.0;
  double effective_mass_dd = 0.0;
  double eff_mass_diff_checker = 0.0;

  Eigen::Vector3d hammer_tip_actual_velocity_vector = {0, 0, 0};
  Eigen::Vector3d hammer_tip_actual_position_vector = {0, 0, 0};
  Eigen::Vector3d hammer_tip_actual_position_vector_realrobot = {0, 0, 0};
  Eigen::Vector3d hammer_tip_position_observer_error = {0, 0, 0};
  Eigen::Vector3d floating_base_position_observer_error = {0, 0, 0};

  Eigen::Vector3d hammer_tip_reference_velocity_vector = {0, 0, 0};
  Eigen::Vector3d hammer_tip_reference_position_vector = {0, 0, 0};
  Eigen::Vector3d bspline_tracking_error = {0, 0, 0};
  Eigen::VectorXd bspline_eval;
  double bspline_eval_norm = 0.0;
  double projected_momentum_of_hammer_tip = 0.0;
  double vector_orientation_error = 0.0;

  std::vector<double> qd;
  Eigen::VectorXd qdm;
  mc_filter::LowPass<Eigen::VectorXd> qd_filter_{0.005, 0.03183};
  mc_filter::LowPass<Eigen::VectorXd> qdm_filter_{0.005, 0.03183};
  mc_filter::LowPass<Eigen::VectorXd> tau_imp_derivate_filter_{0.005, 0.03183};
  mc_filter::LowPass<Eigen::VectorXd> tau_imp_derivate_qp_filter_{0.005, 0.03183};
  mc_filter::LowPass<Eigen::VectorXd> tau_imp_derivate_act_filter_{0.005, 0.03183};
  mc_filter::LowPass<Eigen::VectorXd> tau_imp_derivate_num_filter_{0.005, 0.03183};
  double filter_cutoff_period_ = 0.03183;
  Eigen::VectorXd qd_previous;
  Eigen::VectorXd tau_imp_true_speed;
  Eigen::VectorXd tau_imp_true_force;
  Eigen::VectorXd tau_imp;
  Eigen::VectorXd tau_imp_act;
  Eigen::VectorXd tau_imp_previous;
  Eigen::VectorXd tau_imp_derivate;
  Eigen::VectorXd tau_imp_derivate_qp;
  Eigen::VectorXd tau_imp_derivate_act;
  Eigen::VectorXd tau_imp_derivate_num;
  Eigen::VectorXd tau_imp_derivate_low_limit;
  Eigen::VectorXd tau_imp_derivate_high_limit;
  Eigen::VectorXd end_effector_velocity;

  double stabilizing_eval_norm = 0.0;
  double stabilizing_speed_norm = 0.0;

  double com_eval_norm = 0.0;
  double pelvis_eval_norm = 0.0;
  double torso_eval_norm = 0.0;
  double contacts_eval_norm = 0.0;

  // Solver constraints & tasks
  std::unique_ptr<mc_solver::DynamicsConstraint> dynamicsConstraint;
  std::unique_ptr<mc_solver::ImpulseConstraint> impulseConstraint;
  std::unique_ptr<mc_solver::ContactConstraint> contactConstraintSet;
  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizerTask;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration stabiConf;

  // Robot sensors & doubles
  std::shared_ptr<mc_rbdyn::Robots> comparisonRobots_;
  const mc_rbdyn::BodySensor & floatingBaseSensor_;

  const std::vector<std::string> mass_maximization_active_joints = {
      "LCY" , "LCR" , "LCP" , "LKP" , "LAP" , "LAR" ,
      "RCY" , "RCR" , "RCP" , "RKP" , "RAP" , "RAR" ,
      "WP"  , "WR"  , "WY"  , "HY"  , "HP"  ,
      "LSC" , "LSP" , "LSR" , "LSY" , "LEP" , "LWRY", "LWRR", "LWRP", "LHDY",
      "RSC" , "RSP" , "RSR" , "RSY" , "REP" , "RWRY", "RWRR", "RWRP", "RHDY"
  };

  std::string selected_plot_joint_ = "LWRR";
  std::string selected_plot_mode_ = "Impulsive Torque";
  const std::vector<std::string> plot_modes_ = {"Impulsive Torque", "Derivative of Impulsive Torque"};

  double plot_timer_ = 0.0;
  bool should_plot_tick_ = false;

  // Joint friction model & compensation
  bool enable_friction_compensation_ = false;
  std::vector<double> sim_friction_torques_;
  std::vector<double> joint_friction_torques_;
};