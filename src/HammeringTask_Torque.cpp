#include "HammeringTask_Torque.h"
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/Jacobian.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/ContactConstraint.h>

HammeringTask_Torque::HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM),
  floatingBaseSensor_(robot().bodySensor("FloatingBase"))
{
  config_.load(config);
  load_parameters();

  datastore().make<std::string>("ControlMode", params_.control_mode);
  datastore().make<std::string>("Coriolis", "Yes"); 

  nail_rot = robot(params_.nail_robot_name).frame(params_.nail_frame).position().rotation();
  nail_normal_vector_world_frame = (nail_rot.transpose() * normal_vector_nail_frame).normalized();

  // Contact constraint for feet on ground (acceleration type)
  contactConstraintSet = std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintSet);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // Dynamics constraint
  dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(
      robots(), robot().robotIndex(), solver().dt(),
      params_.impulse.damping, params_.impulse.velocity_percentage, params_.impulse.infTorque, true);
  solver().addConstraintSet(dynamicsConstraint);

  mc_rtc::log::info("[HammeringTask_Torque] Nail normal vector in world frame: {}", nail_normal_vector_world_frame.transpose());

  // LIPM Stabilizer Task
  stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.copMaxVel = {{3., 3., 3.}, {0.1, 0.1, 0.1}};

  stabilizerTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
      solver().robots(),
      solver().realRobots(),
      robot().robotIndex(),
      stabiConf.leftFootSurface,
      stabiConf.rightFootSurface,
      stabiConf.torsoBodyName,
      solver().dt());

  stabilizerTask->reset();
  stabilizerTask->configure(stabiConf);
  solver().addTask(stabilizerTask);

  apply_parameters();

  auto ext_wrench_conf = stabilizerTask->externalWrenchConfiguration();
  ext_wrench_conf.addExpectedCoMOffset = true;
  ext_wrench_conf.modifyCoMErr = true;
  ext_wrench_conf.modifyZMPErr = true;
  stabilizerTask->externalWrenchConfiguration(ext_wrench_conf);

  qd_previous = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_act = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_low_limit = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_high_limit = Eigen::VectorXd::Zero(robot().mb().nrDof());

  comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());

  add_logs();
  addToGUI();
  mc_rtc::log::success("HammeringTask_Torque initialized with LIPM stabilizer and torque control.");
}

bool HammeringTask_Torque::run()
{
  comparisonRobots_->robot().mbc().q = realRobot().mbc().q;

  // Set floating base pose and velocities from body sensor ground truth
  comparisonRobots_->robot().posW(sva::PTransformd(floatingBaseSensor_.orientation(), floatingBaseSensor_.position()));
  comparisonRobots_->robot().velW(sva::MotionVecd(floatingBaseSensor_.angularVelocity(), floatingBaseSensor_.linearVelocity()));
  comparisonRobots_->robot().accW(sva::MotionVecd(floatingBaseSensor_.angularAcceleration(), floatingBaseSensor_.linearAcceleration()));

  comparisonRobots_->robot().forwardKinematics();
  comparisonRobots_->robot().forwardVelocity();
  comparisonRobots_->robot().forwardAcceleration();
  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  hammer_tip_actual_position_vector_realrobot = comparisonRobots_->robot().frame(params_.hammer_head_frame).position().translation();
  hammer_tip_actual_position_vector = robot().frame(params_.hammer_head_frame).position().translation();
  hammer_tip_actual_velocity_vector = robot().frame(params_.hammer_head_frame).velocity().linear();

  hammer_tip_position_observer_error = hammer_tip_actual_position_vector - hammer_tip_actual_position_vector_realrobot;
  floating_base_position_observer_error = comparisonRobots_->robot().posW().translation() - robot().posW().translation();

  stabilizing_speed_norm = stabilizerTask->speed().norm();
  stabilizing_eval_norm = stabilizerTask->eval().norm();

  // Jacobians for hammer head
  rbd::Jacobian jac(robot().mb(), params_.hammer_head_frame);
  Eigen::MatrixXd world_frame_jacobian = jac.jacobian(robot().mb(), robot().mbc());

  Eigen::MatrixXd full_world_frame_jacobian(6, robot().mb().nrDof());
  Eigen::MatrixXd & J_ = full_world_frame_jacobian;
  jac.fullJacobian(robot().mb(), world_frame_jacobian, J_);

  const auto & world_frame_jacobian_dot = jac.jacobianDot(robot().mb(), robot().mbc());
  Eigen::MatrixXd full_world_frame_jacobian_dot(6, robot().mb().nrDof());
  jac.fullJacobian(robot().mb(), world_frame_jacobian_dot, full_world_frame_jacobian_dot);
  Eigen::MatrixXd & J_d = full_world_frame_jacobian_dot;

  // Sensor frame jacobian
  rbd::Jacobian jac_sensor(robot().mb(), "Larm_Link6");
  Eigen::MatrixXd world_frame_jacobian_Larm_sensor = jac_sensor.jacobian(robot().mb(), robot().mbc());
  Eigen::MatrixXd J_Larm_sensor_(6, robot().mb().nrDof());
  jac_sensor.fullJacobian(robot().mb(), world_frame_jacobian_Larm_sensor, J_Larm_sensor_);

  Eigen::Matrix3d P_n_sub = (nail_normal_vector_world_frame * nail_normal_vector_world_frame.transpose()).normalized();
  Eigen::MatrixXd linear_jacobian = J_.bottomRows(3);

  P_n = Eigen::Matrix<double, 6, 6>::Zero();
  P_n.block<3, 3>(3, 3) = P_n_sub;
  Eigen::VectorXd q_d = tvm::dot(robot().tvmRobot().q(), 1)->value();
  Eigen::VectorXd q_dd = tvm::dot(robot().tvmRobot().q(), 2)->value();

  qd = robot().encoderVelocities();
  qdm = Eigen::VectorXd::Zero(robot().mb().nrDof());

  if(qd.size() >= 44)
  {
    qdm(0) = 0.0; qdm(1) = 0.0; qdm(2) = 0.0; qdm(3) = 0.0; qdm(4) = 0.0; qdm(5) = 0.0;
    qdm(6) = qd.at(6);   // LCY
    qdm(7) = qd.at(7);   // LCR
    qdm(8) = qd.at(8);   // LCP
    qdm(9) = qd.at(9);   // LKP
    qdm(10) = qd.at(10); // LAP
    qdm(11) = qd.at(11); // LAR
    qdm(12) = qd.at(0);  // RCY
    qdm(13) = qd.at(1);  // RCR
    qdm(14) = qd.at(2);  // RCP
    qdm(15) = qd.at(3);  // RKP
    qdm(16) = qd.at(4);  // RAP
    qdm(17) = qd.at(5);  // RAR
    qdm(18) = qd.at(12); // WP
    qdm(19) = qd.at(13); // WR
    qdm(20) = qd.at(14); // WY
    qdm(21) = qd.at(15); // HY
    qdm(22) = qd.at(16); // HP
    qdm(23) = qd.at(17); // LSC
    qdm(24) = qd.at(18); // LSP
    qdm(25) = qd.at(19); // LSR
    qdm(26) = qd.at(20); // LSY
    qdm(27) = qd.at(21); // LEP
    qdm(28) = qd.at(22); // LWRY
    qdm(29) = qd.at(23); // LWRR
    qdm(30) = qd.at(24); // LWRP
    qdm(31) = qd.at(25); // LHDY
    qdm(32) = qd.at(26); // RSC
    qdm(33) = qd.at(27); // RSP
    qdm(34) = qd.at(28); // RSR
    qdm(35) = qd.at(29); // RSY
    qdm(36) = qd.at(30); // REP
    qdm(37) = qd.at(31); // RWRY
    qdm(38) = qd.at(32); // RWRR
    qdm(39) = qd.at(33); // RWRP
    qdm(40) = qd.at(34); // RHDY
  }

  effective_mass = compute_effective_mass_with_mbc(robot().mbc(), *this, nail_normal_vector_world_frame);
  effective_mass_d = compute_effective_mass_d_with_mbc(robot().mbc(), *this, nail_normal_vector_world_frame, effective_mass);
  tau_imp_true_speed = (J_.transpose() * effective_mass * P_n * J_) * solver().dt();

  Eigen::Matrix3d R = Eigen::AngleAxisd(-M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tau_imp_true_force = J_Larm_sensor_.transpose() * P_n_sub * (R * robot().forceSensor("LeftHandForceSensor").force());

  double dt_param = params_.impulse.delta_t > 0.0 ? params_.impulse.delta_t : 0.001;
  double c_res_param = params_.impulse.c_res;

  tau_imp = (-1.0f * (c_res_param + 1.0) / dt_param) * J_.transpose() * effective_mass * P_n * J_ * q_d;
  tau_imp_act = (-1.0f * (c_res_param + 1.0) / dt_param) * J_.transpose() * effective_mass * P_n * J_ * qdm;
  tau_imp_derivate = -(c_res_param + 1.0) / dt_param * ((J_d.transpose() * effective_mass * P_n * J_
      + J_.transpose() * effective_mass_d * P_n * J_
      + J_.transpose() * effective_mass * P_n * J_d) * q_d
      + (J_.transpose() * effective_mass * P_n * J_) * q_dd);
  tau_imp_derivate_num = (tau_imp_act - tau_imp_previous) / solver().dt();
  tau_imp_previous = tau_imp_act;

  end_effector_velocity = linear_jacobian * q_d;

  if(impulseConstraint)
  {
    double cur_dist = (robot().frame(params_.hammer_head_frame).position().translation() -
                       robot(params_.nail_robot_name).frame(params_.nail_frame).position().translation()).norm();
    if(cur_dist < params_.impulse.activation_height)
    {
      double denom = params_.impulse.activation_height * (1.0 - params_.impulse.K);
      if(std::abs(denom) > 1e-6)
      {
        tau_imp_derivate_low_limit = (robot().tvmRobot().limits().tl - tau_imp_act) * impulseConstraint->EffectiveLambda()
            - robot().tvmRobot().limits().tl * (1.0 - params_.impulse.tau_high_multiplier) / denom * linear_jacobian * q_d;
        tau_imp_derivate_high_limit = (robot().tvmRobot().limits().tu - tau_imp_act) * impulseConstraint->EffectiveLambda()
            - robot().tvmRobot().limits().tu * (1.0 - params_.impulse.tau_high_multiplier) / denom * linear_jacobian * q_d;
      }
    }
    else
    {
      tau_imp_derivate_low_limit = (robot().tvmRobot().limits().tl - tau_imp_act) * impulseConstraint->EffectiveLambda();
      tau_imp_derivate_high_limit = (robot().tvmRobot().limits().tu - tau_imp_act) * impulseConstraint->EffectiveLambda();
    }
  }
  else
  {
    tau_imp_derivate_low_limit = robot().tvmRobot().limits().tl * params_.impulse.limit_multiplier;
    tau_imp_derivate_high_limit = robot().tvmRobot().limits().tu * params_.impulse.limit_multiplier;
  }

  qd_previous = qdm;
  total_time_elapsed += solver().dt();
  plot_timer_ += solver().dt();
  if(plot_timer_ >= params_.plot_dt)
  {
    should_plot_tick_ = true;
    plot_timer_ = 0.0;
  }
  else
  {
    should_plot_tick_ = false;
  }

  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void HammeringTask_Torque::reset(const mc_control::ControllerResetData & reset_data)
{
  total_time_elapsed = 0.0;
  plot_timer_ = 0.0;
  should_plot_tick_ = false;
  number_of_hits = 0;
  trajectories_executed = 0;
  impulsive_constraint_flag = false;
  force_felt = false;
  impact_detected = false;
  bspline_active_ = false;
  params_.impulse.activation_height = 0.0;

  if(impulseConstraint)
  {
    solver().removeConstraintSet(*impulseConstraint);
    impulseConstraint.reset();
  }

  comparisonRobots_ = mc_rbdyn::loadRobot(robot().module());
  mc_control::fsm::Controller::reset(reset_data);
  stabilizerTask->reset();
  apply_parameters();

  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  mc_rtc::log::info("[HammeringTask_Torque] Controller reset complete.");
}

int HammeringTask_Torque::get_dof(const std::string & jname) const
{
  if(robot().hasJoint(jname))
  {
    auto jIndex = robot().jointIndexByName(jname);
    return robot().mb().jointPosInDof(jIndex);
  }
  return 29; // default to LWRR
}

double HammeringTask_Torque::compute_effective_mass_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d & normal_vector)
{
  HammeringTask_Torque & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  rbd::Jacobian jac(ctl.robot().mb(), ctl.params_.hammer_head_frame);
  const auto & world_frame_jacobian = jac.jacobian(ctl.robot().mb(), mbc);
  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(ctl.robot().mb(), world_frame_jacobian, full_world_frame_jacobian);
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  rbd::ForwardDynamics fd(ctl.robot().mb());
  fd.computeH(ctl.robot().mb(), mbc);
  Eigen::MatrixXd M = fd.H();
  Eigen::MatrixXd Mi = M.inverse();

  return 1.0 / (normal_vector.transpose() * linear_jacobian * Mi * linear_jacobian.transpose() * normal_vector);
}

double HammeringTask_Torque::compute_effective_mass_d_with_mbc(
  rbd::MultiBodyConfig mbc, 
  mc_control::fsm::Controller & ctl_, 
  const Eigen::Vector3d & normal_vector,
  double eff_mass)
{
  HammeringTask_Torque & ctl = static_cast<HammeringTask_Torque &>(ctl_);
  rbd::Jacobian jac(ctl.robot().mb(), ctl.params_.hammer_head_frame);
  const auto & world_frame_jacobian = jac.jacobian(ctl.robot().mb(), mbc);
  Eigen::MatrixXd full_world_frame_jacobian(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(ctl.robot().mb(), world_frame_jacobian, full_world_frame_jacobian);
  Eigen::MatrixXd linear_jacobian = full_world_frame_jacobian.bottomRows(3);

  const auto & world_frame_jacobian_dot = jac.jacobianDot(ctl.robot().mb(), mbc);
  Eigen::MatrixXd full_world_frame_jacobian_dot(6, ctl.robot().mb().nrDof());
  jac.fullJacobian(ctl.robot().mb(), world_frame_jacobian_dot, full_world_frame_jacobian_dot);
  Eigen::MatrixXd linear_jacobiand = full_world_frame_jacobian_dot.bottomRows(3);

  rbd::ForwardDynamics fd(ctl.robot().mb());
  fd.computeH(ctl.robot().mb(), mbc);
  Eigen::MatrixXd M = fd.H();
  Eigen::MatrixXd Mi = M.inverse();

  rbd::Coriolis coriolis(ctl.robot().mb());
  Eigen::MatrixXd C = coriolis.coriolis(ctl.robot().mb(), mbc);
  Eigen::MatrixXd M_d = C + C.transpose();

  return -1.0 * static_cast<double>(normal_vector.transpose() * (
      linear_jacobiand * Mi * linear_jacobian.transpose() -
      linear_jacobian * Mi * M_d * Mi * linear_jacobian.transpose() +
      linear_jacobian * Mi * linear_jacobiand.transpose()) * normal_vector) * eff_mass * eff_mass;
}
