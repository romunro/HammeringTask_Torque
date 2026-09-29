#pragma once

#include <Eigen/Core>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <string>
#include <array>
#include <vector>

struct StabilizerParams
{
  double torso_stiffness = 50.0;
  double torso_damping = 25.0;
  double torso_weight = 300.0;
  double torso_pitch = -0.05;

  double pelvis_stiffness = 1000.0;
  double pelvis_damping = 100.0;
  double pelvis_weight = 2000.0;

  Eigen::Vector2d dcm_p = {3.0, 3.0};
  Eigen::Vector2d dcm_i = {1.5, 1.5};
  Eigen::Vector2d dcm_d = {1.2, 1.2};

  Eigen::Vector3d com_stiffness = {1000.0, 1000.0, 500.0};
  Eigen::Vector3d com_damping = {70.0, 70.0, 50.0};
  double com_weight = 1000.0;
  Eigen::Vector3d com_dim_weight = {1.0, 1.0, 0.5};
  double com_height = 0.94;
  bool has_com_height = true;

  double contact_weight = 100000.0;
  sva::MotionVecd contact_stiffness = sva::MotionVecd({1.0, 1.0, 1.0}, {1.0, 1.0, 1.0});
  sva::MotionVecd contact_damping = sva::MotionVecd({400.0, 400.0, 400.0}, {400.0, 400.0, 400.0});
  Eigen::Vector2d contact_admittance = {0.003, 0.003};
  mc_rbdyn::Gains3d df_admittance = {0.0, 0.0, 0.0001};
  mc_rbdyn::Gains3d df_damping = {0.0, 0.0, 1.0};
};

struct PostureParams
{
  double base_weight = 500.0;
  double base_stiffness = 100.0;
  double base_damping = 20.0;

  double arm_nullspace_weight = 0.01;
  double arm_nullspace_stiffness = 100.0;
  double arm_nullspace_damping = 20.0;
};

struct TrajectoryParams
{
  double duration = 2.0;
  double waypoint_height = 0.4;
  double stiffness = 100.0;
  double damping = 20.0;
  double weight = 2000.0;
  Eigen::Vector6d dimweights = (Eigen::Vector6d() << 0, 0, 0, 1, 1, 1).finished();
  Eigen::Vector3d init_velocity = {0, 0, 0};
  Eigen::Vector3d final_velocity = {0, 0, -0.5};
  Eigen::Vector3d nail_target_position = {0.4, 0.4, 0.9};
  double effective_mass_maximization_weight = 0.0;
};

struct VectorOrientationParams
{
  double weight = 200.0;
  double stiffness = 40.0;
  double damping = 12.0;
};

struct ImpulseParams
{
  double c_res = 1.0;
  double delta_t = 0.001;
  double limit_multiplier = 1.0;
  double lambda_high = 60.0;
  double lambda_low = 25.0;
  double tau_high_multiplier = 1.5;
  double K = 0.5;
  double activation_height = 0.0;
  bool linear_impulsive_torque_ctr_flag = true;
  std::array<double, 3> damping = {0.1, 0.01, 0.5};
  double velocity_percentage = 0.9;
  bool infTorque = true;
  double filter_cutoff_frequency = 5.0;
};

struct ImpactDetectionParams
{
  double nail_force_threshold = 100.0;
  double sensor_force_threshold = 250.0;
  int max_hits = 1;
};

struct FrictionParams
{
  bool enable = false;
  double tau_c = 0.5;   // Coulomb friction torque [Nm]
  double f_v = 0.0;     // Viscous friction coefficient [Nm/(rad/s)]
  double v_th = 0.02;   // Velocity smoothing threshold [rad/s]
  bool use_ref_vel = true; // Use reference/controller velocity (pure feedforward)
  std::vector<std::string> joints; // Joints to compensate (empty = all)
};

struct ControllerParams
{
  std::string control_mode = "Torque";
  double timestep = 0.005;
  std::string hammer_head_frame = "Hammer_head";
  std::string nail_frame = "nail";
  std::string nail_robot_name = "nail";
  std::string main_robot_name = "hrp5_p";

  std::string stop_hammering_button_name = "Stop hammering";
  std::string linear_constraint_button_name = "Linear constraint activation";
  double plot_dt = 0.05;
  std::string plot_joint = "LWRR";

  bool bezier_curve_verbose = false;
  bool jacobian_verbose = false;
  bool enable_friction_compensation = false;

  StabilizerParams stabilizer;
  PostureParams posture;
  TrajectoryParams trajectory;
  VectorOrientationParams vector_orientation;
  ImpulseParams impulse;
  ImpactDetectionParams impact;
  FrictionParams friction;

  void load(const mc_rtc::Configuration & config);
};
