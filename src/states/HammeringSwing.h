#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <ndcurves/curve_constraint.h>
#include <RBDyn/MultiBodyConfig.h>
#include <unordered_map>
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

  inline static const std::unordered_map<std::string, int> joint_index_map = {
      {"LCY" , 0},
      {"LCR" , 1},
      {"LCP" , 2},
      {"LKP" , 3},
      {"LAP" , 4},
      {"LAR" , 5},
      {"RCY" , 6},
      {"RCR" , 7},
      {"RCP" , 8},
      {"RKP" , 9},
      {"RAP" , 10},
      {"RAR" , 11},
      {"WP"  , 12},
      {"WR"  , 13},
      {"WY"  , 14},
      {"HY"  , 15},
      {"HP"  , 16},
      {"LSC" , 17},
      {"LSP" , 18},
      {"LSR" , 19},
      {"LSY" , 20},
      {"LEP" , 21},
      {"LWRY", 22},
      {"LWRR", 23},
      {"LWRP", 24},
      {"LHDY", 25},
      {"RSC" , 26},
      {"RSP" , 27},
      {"RSR" , 28},
      {"RSY" , 29},
      {"REP" , 30},
      {"RWRY", 31},
      {"RWRR", 32},
      {"RWRP", 33},
      {"RHDY", 34}
  };

  static int jointIndex(const std::string & name)
  {
      auto it = joint_index_map.find(name);
      return (it != joint_index_map.end()) ? it->second : -1;
  }

  const std::vector<std::string> mass_maximization_active_joints = {
      "LSC", "LSP", "LSR", "LSY", "LEP", "LWRY", "LWRR", "LWRP", "LHDY"
  };

  Eigen::VectorXd _gradient_of_m;
  rbd::MultiBodyConfig _new_mbc;

  double vector_error(const Eigen::Vector3d & va, const Eigen::Vector3d & vb) const;
  Eigen::Vector3d bezier_vel_from_task(const std::shared_ptr<mc_tasks::BSplineTrajectoryTask> & task, double dt) const;
  void add_logs(mc_control::fsm::Controller & ctl);
  void rm_logs(mc_control::fsm::Controller & ctl);
  /**
    @brief Compute the effective mass using the mbc object (rbd::MultiBodyConfiguration)
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
  */
  const double compute_effective_mass_with_mbc(rbd::MultiBodyConfig mbc, 
                                              mc_control::fsm::Controller & ctl_, 
                                              const Eigen::Vector3d &normal_vector) const;
  /**
    @brief Compute the derivative of the effective mass with respect to the robot configuration
          using backward difference
    @param mbc the MultiBodyConfig of the robot
    @param ctl_
    @param normal_vector the vector used to compute the effective mass of the robot
  */  
  Eigen::VectorXd compute_emass_gradient_backward_difference_mbc(const rbd::MultiBodyConfig &mbc,
                                                                mc_control::fsm::Controller &ctl_,
                                                                const Eigen::Vector3d &normal_vector,
                                                                const std::vector<std::string> &active_joints) const;
};
