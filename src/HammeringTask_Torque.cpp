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

  double fc = params_.impulse.filter_cutoff_frequency > 0.0 ? params_.impulse.filter_cutoff_frequency : 5.0;
  filter_cutoff_period_ = 1.0 / (2.0 * M_PI * fc);
  qd_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);
  qdm_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);
  tau_imp_derivate_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);
  tau_imp_derivate_qp_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);
  tau_imp_derivate_act_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);
  tau_imp_derivate_num_filter_ = mc_filter::LowPass<Eigen::VectorXd>(solver().dt(), filter_cutoff_period_);

  qd_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  qdm_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_qp_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_act_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_num_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  mc_rtc::log::info("[HammeringTask_Torque] mc_filter::LowPass initialized: fc = {} Hz, cutoffPeriod = {} s, dt = {} s",
                    fc, filter_cutoff_period_, solver().dt());

  qd_previous = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_act = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_previous = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_qp = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_act = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_num = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_derivate_low_limit = Eigen::VectorXd::Constant(robot().mb().nrDof(), std::numeric_limits<double>::quiet_NaN());
  tau_imp_derivate_high_limit = Eigen::VectorXd::Constant(robot().mb().nrDof(), std::numeric_limits<double>::quiet_NaN());
  tau_imp_true_speed = Eigen::VectorXd::Zero(robot().mb().nrDof());
  tau_imp_true_force = Eigen::VectorXd::Zero(robot().mb().nrDof());

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
  qd_filter_.update(tvm::dot(robot().tvmRobot().q(), 1)->value());
  const Eigen::VectorXd & q_d = qd_filter_.eval();
  Eigen::VectorXd q_dd = tvm::dot(robot().tvmRobot().q(), 2)->value();

  auto current_vel = robot().encoderVelocities();
  qdm = Eigen::VectorXd::Zero(robot().mb().nrDof());
  qdm(0) = 0.0;
  qdm(1) = 0.0;
  qdm(2) = 0.0;
  qdm(3) = 0.0;
  qdm(4) = 0.0;
  qdm(5) = 0.0;
  qdm(6) = current_vel.at(6);//LCY
  qdm(7) = current_vel.at(7);//LCR
  qdm(8) = current_vel.at(8);//LCP
  qdm(9) = current_vel.at(9);//LKP
  qdm(10) = current_vel.at(10);//LAP
  qdm(11) = current_vel.at(11);//LAR
  qdm(12) = current_vel.at(0);//RCY
  qdm(13) = current_vel.at(1);//RCR
  qdm(14) = current_vel.at(2);//RCP
  qdm(15) = current_vel.at(3);//RKP
  qdm(16) = current_vel.at(4);//RAP
  qdm(17) = current_vel.at(5);//RAR
  qdm(18) = current_vel.at(12);//WP
  qdm(19) = current_vel.at(13);//WR
  qdm(20) = current_vel.at(14);//WY
  qdm(21) = current_vel.at(15);//HY
  qdm(22) = current_vel.at(16);//HP
  qdm(23) = current_vel.at(35);//LSC
  qdm(24) = current_vel.at(36);//LSP
  qdm(25) = current_vel.at(37);//LSR
  qdm(26) = current_vel.at(38);//LSY
  qdm(27) = current_vel.at(39);//LEP
  qdm(28) = current_vel.at(40);//LWRY
  qdm(29) = current_vel.at(41);//LWRR
  qdm(30) = current_vel.at(42);//LWRP
  qdm(31) = current_vel.at(43);//LHDY
  qdm(32) = current_vel.at(17);//RSC
  qdm(33) = current_vel.at(18);//RSP
  qdm(34) = current_vel.at(19);//RSR
  qdm(35) = current_vel.at(20);//RSY
  qdm(36) = current_vel.at(21);//REP
  qdm(37) = current_vel.at(22);//RWRY
  qdm(38) = current_vel.at(23);//RWRR
  qdm(39) = current_vel.at(24);//RWRP
  qdm(40) = current_vel.at(25);//RHDY

  qdm_filter_.update(qdm);
  const Eigen::VectorXd & qdm_filtered = qdm_filter_.eval();

  effective_mass = compute_effective_mass_with_mbc(robot().mbc(), *this, nail_normal_vector_world_frame);
  effective_mass_d = compute_effective_mass_d_with_mbc(robot().mbc(), *this, nail_normal_vector_world_frame, effective_mass);
  double dt_param = params_.impulse.delta_t > 0.0 ? params_.impulse.delta_t : 0.001;
  double c_res_param = params_.impulse.c_res;

  tau_imp_true_speed = (-1.0f * (c_res_param + 1.0) / dt_param) * J_.transpose() * effective_mass * P_n * J_ * qdm_filtered;

  Eigen::Matrix3d R = Eigen::AngleAxisd(-M_PI / 4.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  tau_imp_true_force = J_Larm_sensor_.transpose() * P_n_sub * (R * robot().forceSensor("LeftHandForceSensor").force());

  // Floating base DOFs (0..5) are unactuated and do not contribute to joint impulsive torques
  Eigen::VectorXd q_d_arm = q_d;
  q_d_arm.head<6>().setZero();
  Eigen::VectorXd q_dd_arm = q_dd;
  q_dd_arm.head<6>().setZero();

  tau_imp = (-1.0f * (c_res_param + 1.0) / dt_param) * J_.transpose() * effective_mass * P_n * J_ * q_d_arm;
  tau_imp_act = (-1.0f * (c_res_param + 1.0) / dt_param) * J_.transpose() * effective_mass * P_n * J_ * qdm_filtered;

  Eigen::VectorXd raw_tau_imp_derivate_qp = -(c_res_param + 1.0) / dt_param * ((J_d.transpose() * effective_mass * P_n * J_
      + J_.transpose() * effective_mass_d * P_n * J_
      + J_.transpose() * effective_mass * P_n * J_d) * q_d_arm
      + (J_.transpose() * effective_mass * P_n * J_) * q_dd_arm);
  tau_imp_derivate_qp_filter_.update(raw_tau_imp_derivate_qp);
  tau_imp_derivate_qp = tau_imp_derivate_qp_filter_.eval();
  tau_imp_derivate = tau_imp_derivate_qp;

  Eigen::VectorXd qdm_d = (qdm_filtered - qd_previous) / solver().dt();
  Eigen::VectorXd raw_tau_imp_derivate_act = -(c_res_param + 1.0) / dt_param * ((J_d.transpose() * effective_mass * P_n * J_
      + J_.transpose() * effective_mass_d * P_n * J_
      + J_.transpose() * effective_mass * P_n * J_d) * qdm_filtered
      + (J_.transpose() * effective_mass * P_n * J_) * qdm_d);
  tau_imp_derivate_act_filter_.update(raw_tau_imp_derivate_act);
  tau_imp_derivate_act = tau_imp_derivate_act_filter_.eval();

  Eigen::VectorXd raw_tau_imp_derivate_num = (tau_imp_act - tau_imp_previous) / solver().dt();
  tau_imp_derivate_num_filter_.update(raw_tau_imp_derivate_num);
  tau_imp_derivate_num = tau_imp_derivate_num_filter_.eval();
  tau_imp_previous = tau_imp_act;

  end_effector_velocity = linear_jacobian * q_d;

  if(impulseConstraint && impulsive_constraint_flag && impulseConstraint->EffectiveLambda().size() == robot().mb().nrDof())
  {
    tau_imp = impulseConstraint->TorquePrediction();
    tau_imp_derivate_low_limit = impulseConstraint->RightSideLower();
    tau_imp_derivate_high_limit = impulseConstraint->RightSideUpper();
    tau_imp_derivate_qp = impulseConstraint->DerivativeQP();
  }
  else
  {
    tau_imp_derivate_low_limit.setConstant(std::numeric_limits<double>::quiet_NaN());
    tau_imp_derivate_high_limit.setConstant(std::numeric_limits<double>::quiet_NaN());
  }
  tau_imp_derivate = tau_imp_derivate_qp;

  qd_previous = qdm_filtered;
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

  qd_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  qdm_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_qp_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_act_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  tau_imp_derivate_num_filter_.reset(Eigen::VectorXd::Zero(robot().mb().nrDof()));
  if(tau_imp_previous.size() == robot().mb().nrDof())
  {
    tau_imp.setZero();
    tau_imp_act.setZero();
    tau_imp_previous.setZero();
    tau_imp_derivate.setZero();
    tau_imp_derivate_qp.setZero();
    tau_imp_derivate_act.setZero();
    tau_imp_derivate_num.setZero();
    tau_imp_derivate_low_limit.setConstant(std::numeric_limits<double>::quiet_NaN());
    tau_imp_derivate_high_limit.setConstant(std::numeric_limits<double>::quiet_NaN());
    tau_imp_true_speed.setZero();
    tau_imp_true_force.setZero();
  }

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
