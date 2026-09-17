#include "HammeringTask_Torque.h"
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

HammeringTask_Torque::HammeringTask_Torque(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_.load(config);
  load_parameters();

  datastore().make<std::string>("ControlMode", control_mode);
  datastore().make<std::string>("Coriolis", "Yes");

  // 1. Contact constraint for feet on ground (acceleration type)
  contactConstraintSet = std::make_unique<mc_solver::ContactConstraint>(timeStep, mc_solver::ContactConstraint::ContactType::Acceleration);
  solver().addConstraintSet(contactConstraintSet);
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});

  // 2. Dynamics constraint
  dynamicsConstraint = std::make_unique<mc_solver::DynamicsConstraint>(robots(), robot().robotIndex(), solver().dt(), _damping, _vp, _infTorque, true);
  solver().addConstraintSet(dynamicsConstraint);

  // 3. Posture task
  auto postureTask = getPostureTask(robot().name());
  if(postureTask)
  {
    if(!postureTask->inSolver())
    {
      solver().addTask(postureTask);
    }
  }

  // 4. LIPM Stabilizer Task
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

  auto & Active_tasks = solver().tasks();
  for (auto i : Active_tasks){
    mc_rtc::log::info("This controller has task: {} of type: {}", i->name(), i->type());
  }
  addToGUI();
  mc_rtc::log::success("HammeringTask_Torque init done with LIPM stabilizer and posture task.");
}

bool HammeringTask_Torque::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void HammeringTask_Torque::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  stabilizerTask->reset();
  apply_parameters();
}

void HammeringTask_Torque::load_parameters()
{
  if(config_.has("ControlMode"))
  {
    config_("ControlMode", control_mode);
  }

  std::string robot_key = "hrp5_p";
  if(config_.has(robot_key) && config_(robot_key).has("posture"))
  {
    auto post = config_(robot_key)("posture");
    if(post.has("stiffness")) post("stiffness", base_posture_stiffness);
    if(post.has("weight")) post("weight", base_posture_weight);
  }

  std::string global_controller = "global_controller_params";
  if(!config_.has(global_controller)) return;
  auto global_params = config_(global_controller);

  if(global_params.has("hitting_constraint_paramater"))
  {
    auto hc = global_params("hitting_constraint_paramater");
    if(hc.has("damping")) _damping = hc("damping");
    if(hc.has("velocity_percentage")) _vp = hc("velocity_percentage");
    if(hc.has("infTorque")) _infTorque = hc("infTorque");
  }

  if(global_params.has("stabilizer"))
  {
    auto stab = global_params("stabilizer");
    if(stab.has("torso"))
    {
      if(stab("torso").has("stiffness")) stab("torso")("stiffness", _torso_task_stiffness);
      if(stab("torso").has("weight")) stab("torso")("weight", _torso_task_weight);
      if(stab("torso").has("pitch")) stab("torso")("pitch", _torso_pitch);
    }
    if(stab.has("pelvis"))
    {
      if(stab("pelvis").has("stiffness")) stab("pelvis")("stiffness", _pelvis_task_stiffness);
      if(stab("pelvis").has("weight")) stab("pelvis")("weight", _pelvis_task_weight);
    }
    if(stab.has("dcm"))
    {
      if(stab("dcm").has("p")) stab("dcm")("p", _dcm_p);
      if(stab("dcm").has("i")) stab("dcm")("i", _dcm_i);
      if(stab("dcm").has("d")) stab("dcm")("d", _dcm_d);
    }
    if(stab.has("com"))
    {
      if(stab("com").has("stiffness")) stab("com")("stiffness", _com_stiffness);
      if(stab("com").has("weight"))
      {
        auto w = stab("com")("weight");
        if(w.isArray())
        {
          std::vector<double> vw = w;
          if(vw.size() == 3)
          {
            _com_dim_weight = Eigen::Vector3d(vw[0], vw[1], vw[2]);
            _com_weight = _com_dim_weight.maxCoeff();
            if(_com_weight > 0)
            {
              _com_dim_weight /= _com_weight;
            }
          }
        }
        else
        {
          _com_weight = static_cast<double>(w);
        }
      }
      if(stab("com").has("dimWeight"))
      {
        stab("com")("dimWeight", _com_dim_weight);
      }
      if(stab("com").has("height"))
      {
        stab("com")("height", _com_height);
        _has_com_height = true;
      }
    }
    if(stab.has("contact"))
    {
      if(stab("contact").has("weight")) stab("contact")("weight", _contact_task_weight);
      if(stab("contact").has("stiffness")) stab("contact")("stiffness", _contact_stiffness);
      if(stab("contact").has("damping")) stab("contact")("damping", _contact_damping);
      if(stab("contact").has("admittance")) stab("contact")("admittance", _contact_admittance);
      if(stab("contact").has("df_admittance"))
      {
        stab("contact")("df_admittance", _df_admittance);
      }
    }
  }
}

void HammeringTask_Torque::apply_parameters()
{
  auto postureTask = getPostureTask(robot().name());
  if(postureTask)
  {
    std::string robot_key = robot().name();
    if(config_.has(robot_key) && config_(robot_key).has("posture"))
    {
      postureTask->load(solver(), config_(robot_key)("posture"));
    }
    else
    {
      postureTask->stiffness(base_posture_stiffness);
      postureTask->weight(base_posture_weight);
    }
  }

  if(stabilizerTask)
  {
    stabilizerTask->torsoStiffness(_torso_task_stiffness);
    stabilizerTask->torsoWeight(_torso_task_weight);
    stabilizerTask->torsoPitch(_torso_pitch);
    stabilizerTask->pelvisStiffness(_pelvis_task_stiffness);
    stabilizerTask->pelvisWeight(_pelvis_task_weight);
    stabilizerTask->dcmGains(_dcm_p, _dcm_i, _dcm_d);
    stabilizerTask->comStiffness(_com_stiffness);
    stabilizerTask->comWeight(_com_weight);
    stabilizerTask->contactWeight(_contact_task_weight);
    stabilizerTask->contactStiffness(_contact_stiffness);
    stabilizerTask->contactDamping(_contact_damping);
    stabilizerTask->copAdmittance(_contact_admittance);

    auto c = stabilizerTask->config();
    c.dfAdmittance = _df_admittance;
    c.torsoPitch = _torso_pitch;
    c.comDimWeight = _com_dim_weight;
    if(_has_com_height)
    {
      c.comHeight = _com_height;
    }
    stabilizerTask->configure(c);

    if(_has_com_height)
    {
      Eigen::Vector3d com = stabilizerTask->targetCoM();
      com.z() = _com_height;
      stabilizerTask->staticTarget(com);
      mc_rtc::log::info("[HammeringTask_Torque] Applied CoM height target: {}", _com_height);
    }
  }
}

void HammeringTask_Torque::addToGUI()
{
  gui()->addElement({"Stabilizer Gains"},
    mc_rtc::gui::NumberInput("Posture Stiffness",
      [this]() { return base_posture_stiffness; },
      [this](double s) {
        base_posture_stiffness = s;
        auto p = getPostureTask(robot().name());
        if(p) p->stiffness(s);
      }),
    mc_rtc::gui::NumberInput("Posture Weight",
      [this]() { return base_posture_weight; },
      [this](double w) {
        base_posture_weight = w;
        auto p = getPostureTask(robot().name());
        if(p) p->weight(w);
      }),
    mc_rtc::gui::NumberInput("Torso Stiffness",
      [this]() { return _torso_task_stiffness; },
      [this](double s) { _torso_task_stiffness = s; stabilizerTask->torsoStiffness(s); }),
    mc_rtc::gui::NumberInput("Torso Weight",
      [this]() { return _torso_task_weight; },
      [this](double w) { _torso_task_weight = w; stabilizerTask->torsoWeight(w); }),
    mc_rtc::gui::NumberInput("Torso Pitch",
      [this]() { return _torso_pitch; },
      [this](double p) { _torso_pitch = p; stabilizerTask->torsoPitch(p); }),
    mc_rtc::gui::NumberInput("Pelvis Stiffness",
      [this]() { return _pelvis_task_stiffness; },
      [this](double s) { _pelvis_task_stiffness = s; stabilizerTask->pelvisStiffness(s); }),
    mc_rtc::gui::NumberInput("Pelvis Weight",
      [this]() { return _pelvis_task_weight; },
      [this](double w) { _pelvis_task_weight = w; stabilizerTask->pelvisWeight(w); }),
    mc_rtc::gui::ArrayInput("CoM Stiffness",
      [this]() -> const Eigen::Vector3d & { return _com_stiffness; },
      [this](const Eigen::Vector3d & s) { _com_stiffness = s; stabilizerTask->comStiffness(s); }),
    mc_rtc::gui::NumberInput("CoM Weight",
      [this]() { return _com_weight; },
      [this](double w) { _com_weight = w; stabilizerTask->comWeight(w); }),
    mc_rtc::gui::NumberInput("CoM Height Target",
      [this]() { return _com_height; },
      [this](double h) {
        _com_height = h;
        _has_com_height = true;
        Eigen::Vector3d com = stabilizerTask->targetCoM();
        com.z() = h;
        stabilizerTask->staticTarget(com);
      }),
    mc_rtc::gui::ArrayInput("CoP Admittance (roll, pitch)",
      [this]() -> const Eigen::Vector2d & { return _contact_admittance; },
      [this](const Eigen::Vector2d & a) { _contact_admittance = a; stabilizerTask->copAdmittance(a); }),
    mc_rtc::gui::ArrayInput("Foot Force Diff Admittance (x, y, z)",
      [this]() -> const mc_rbdyn::Gains3d & { return _df_admittance; },
      [this](const mc_rbdyn::Gains3d & a) {
        _df_admittance = a;
        auto c = stabilizerTask->config();
        c.dfAdmittance = a;
        stabilizerTask->configure(c);
      })
  );
}
