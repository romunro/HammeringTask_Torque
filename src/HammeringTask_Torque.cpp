#include "HammeringTask_Torque.h"

HammeringTask_Torque::HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_.load(config);
  datastore().make<std::string>("ControlMode", "Torque");
  // datastore().make<std::string>("Coriolis", "Yes");

  // Add and configure the posture task                                                                                          
  solver().addTask(postureTask);                                                                                                    
  postureTask->stiffness(40.0);                                                                                                     
  postureTask->weight(100.0); 
  // Stiffen the legs so knees don't buckle                                                                 
  postureTask->jointStiffness(robot(), "LKP", 350.0);                                                                               
  postureTask->jointStiffness(robot(), "RKP", 350.0);                                                                               
  postureTask->jointStiffness(robot(), "LCP", 120.0);                                                                               
  postureTask->jointStiffness(robot(), "RCP", 120.0);                                                                               
  postureTask->jointStiffness(robot(), "LCR", 100.0);                                                                               
  postureTask->jointStiffness(robot(), "RCR", 100.0);                                                                               
  postureTask->jointStiffness(robot(), "LAP", 80.0);                                                                                
  postureTask->jointStiffness(robot(), "RAP", 80.0);                                                                                
  postureTask->jointStiffness(robot(), "WP", 200.0);
  mc_rtc::log::success("HammeringTask_Torque init done ");
}

bool HammeringTask_Torque::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoop);
}

void HammeringTask_Torque::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


