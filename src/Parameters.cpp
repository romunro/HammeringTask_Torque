#include "Parameters.h"
#include "HammeringTask_Torque.h"
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/CoMTask.h>

void ControllerParams::load(const mc_rtc::Configuration & config)
{
  if(config.has("ControlMode")) config("ControlMode", control_mode);

  if(!config.has("global_controller_params"))
  {
    mc_rtc::log::warning("[Parameters] No global_controller_params found in YAML, using defaults.");
    return;
  }

  auto gp = config("global_controller_params");

  if(gp.has("timestep")) gp("timestep", timestep);

  if(gp.has("quality_of_life"))
  {
    auto qol = gp("quality_of_life");
    if(qol.has("bezier_curve_verbose_active")) qol("bezier_curve_verbose_active", bezier_curve_verbose);
    if(qol.has("jacobian_verbose_active")) qol("jacobian_verbose_active", jacobian_verbose);
  }

  if(gp.has("gui"))
  {
    auto g = gp("gui");
    if(g.has("stop_hammering_button_name")) g("stop_hammering_button_name", stop_hammering_button_name);
    if(g.has("linear_constraint_button_name")) g("linear_constraint_button_name", linear_constraint_button_name);
    if(g.has("plot_dt")) g("plot_dt", plot_dt);
    if(g.has("plot_joint")) g("plot_joint", plot_joint);
  }

  if(gp.has("frames"))
  {
    auto f = gp("frames");
    if(f.has("Hammer_Head_frame")) f("Hammer_Head_frame", hammer_head_frame);
    if(f.has("nail")) f("nail", nail_frame);
  }

  if(gp.has("impact_detection"))
  {
    auto m = gp("impact_detection");
    if(m.has("nail_impact_force_threshold")) m("nail_impact_force_threshold", impact.nail_force_threshold);
    if(m.has("sensor_impact_force_threshold")) m("sensor_impact_force_threshold", impact.sensor_force_threshold);
    if(m.has("max_number_of_hits")) m("max_number_of_hits", impact.max_hits);
  }

  if(gp.has("hitting_constraint_paramater"))
  {
    auto hc = gp("hitting_constraint_paramater");
    if(hc.has("c_res")) hc("c_res", impulse.c_res);
    if(hc.has("lambda_high")) hc("lambda_high", impulse.lambda_high);
    if(hc.has("lambda_low")) hc("lambda_low", impulse.lambda_low);
    if(hc.has("delta_t")) hc("delta_t", impulse.delta_t);
    if(hc.has("impulsive_tau_limit_multiplier")) hc("impulsive_tau_limit_multiplier", impulse.limit_multiplier);
    if(hc.has("damping")) hc("damping", impulse.damping);
    if(hc.has("velocity_percentage")) hc("velocity_percentage", impulse.velocity_percentage);
    if(hc.has("Activation_height")) hc("Activation_height", impulse.activation_height);
    if(hc.has("tau_high_mulitplier")) hc("tau_high_mulitplier", impulse.tau_high_multiplier);
    if(hc.has("K")) hc("K", impulse.K);
    if(hc.has("infTorque")) hc("infTorque", impulse.infTorque);
  }

  std::string robot_key = "hrp5_p";
  if(config.has(robot_key) && config(robot_key).has("posture"))
  {
    auto post = config(robot_key)("posture");
    if(post.has("stiffness")) post("stiffness", posture.base_stiffness);
    if(post.has("weight")) post("weight", posture.base_weight);
    if(post.has("damping"))
    {
      post("damping", posture.base_damping);
    }
    else
    {
      posture.base_damping = 2.0 * std::sqrt(posture.base_stiffness);
    }
  }

  if(gp.has("stabilizer"))
  {
    auto stab = gp("stabilizer");
    if(stab.has("torso"))
    {
      if(stab("torso").has("stiffness")) stab("torso")("stiffness", stabilizer.torso_stiffness);
      if(stab("torso").has("weight")) stab("torso")("weight", stabilizer.torso_weight);
      if(stab("torso").has("pitch")) stab("torso")("pitch", stabilizer.torso_pitch);
      stabilizer.torso_damping = stab("torso").has("damping")
                                   ? static_cast<double>(stab("torso")("damping"))
                                   : 2.0 * std::sqrt(stabilizer.torso_stiffness);
    }
    if(stab.has("pelvis"))
    {
      if(stab("pelvis").has("stiffness")) stab("pelvis")("stiffness", stabilizer.pelvis_stiffness);
      if(stab("pelvis").has("weight")) stab("pelvis")("weight", stabilizer.pelvis_weight);
      stabilizer.pelvis_damping = stab("pelvis").has("damping")
                                    ? static_cast<double>(stab("pelvis")("damping"))
                                    : 2.0 * std::sqrt(stabilizer.pelvis_stiffness);
    }
    if(stab.has("dcm"))
    {
      if(stab("dcm").has("p")) stab("dcm")("p", stabilizer.dcm_p);
      if(stab("dcm").has("i")) stab("dcm")("i", stabilizer.dcm_i);
      if(stab("dcm").has("d")) stab("dcm")("d", stabilizer.dcm_d);
    }
    if(stab.has("com"))
    {
      if(stab("com").has("stiffness")) stab("com")("stiffness", stabilizer.com_stiffness);
      if(stab("com").has("damping"))
      {
        stab("com")("damping", stabilizer.com_damping);
      }
      else
      {
        stabilizer.com_damping = 2.0 * stabilizer.com_stiffness.cwiseSqrt();
      }
      if(stab("com").has("weight"))
      {
        auto w = stab("com")("weight");
        if(w.isArray())
        {
          std::vector<double> vw = w;
          if(vw.size() == 3)
          {
            stabilizer.com_dim_weight = Eigen::Vector3d(vw[0], vw[1], vw[2]);
            stabilizer.com_weight = stabilizer.com_dim_weight.maxCoeff();
            if(stabilizer.com_weight > 0) stabilizer.com_dim_weight /= stabilizer.com_weight;
          }
        }
        else
        {
          stabilizer.com_weight = static_cast<double>(w);
        }
      }
      if(stab("com").has("dimWeight")) stab("com")("dimWeight", stabilizer.com_dim_weight);
      if(stab("com").has("height"))
      {
        stab("com")("height", stabilizer.com_height);
        stabilizer.has_com_height = true;
      }
    }
    if(stab.has("contact"))
    {
      if(stab("contact").has("weight")) stab("contact")("weight", stabilizer.contact_weight);
      if(stab("contact").has("stiffness")) stab("contact")("stiffness", stabilizer.contact_stiffness);
      if(stab("contact").has("damping")) stab("contact")("damping", stabilizer.contact_damping);
      if(stab("contact").has("admittance")) stab("contact")("admittance", stabilizer.contact_admittance);
      if(stab("contact").has("df_admittance")) stab("contact")("df_admittance", stabilizer.df_admittance);
      if(stab("contact").has("df_damping")) stab("contact")("df_damping", stabilizer.df_damping);
    }
  }

  if(gp.has("hitting_tasks_paramater"))
  {
    auto ht = gp("hitting_tasks_paramater");
    if(ht.has("nail_target_position")) ht("nail_target_position", trajectory.nail_target_position);
    else if(ht.has("hitting_target")) ht("hitting_target", trajectory.nail_target_position);

    if(ht.has("arm_nullspace_posture_weight")) ht("arm_nullspace_posture_weight", posture.arm_nullspace_weight);
    if(ht.has("arm_nullspace_posture_stiffness")) ht("arm_nullspace_posture_stiffness", posture.arm_nullspace_stiffness);
    posture.arm_nullspace_damping = 2.0 * std::sqrt(posture.arm_nullspace_stiffness);

    if(ht.has("effective_mass_maximization_task_weight")) ht("effective_mass_maximization_task_weight", trajectory.effective_mass_maximization_weight);

    if(ht.has("vector_orientation_task_weight")) ht("vector_orientation_task_weight", vector_orientation.weight);
    if(ht.has("vector_orientation_task_stiffness")) ht("vector_orientation_task_stiffness", vector_orientation.stiffness);
    if(ht.has("vector_orientation_task_damping")) ht("vector_orientation_task_damping", vector_orientation.damping);
    else vector_orientation.damping = 2.0 * std::sqrt(vector_orientation.stiffness);

    double rx = 0.0, ry = 0.0, rz = 0.0, tx = 1.0, ty = 1.0, tz = 1.0;
    if(ht.has("bspline_dimweight_rx")) ht("bspline_dimweight_rx", rx);
    if(ht.has("bspline_dimweight_ry")) ht("bspline_dimweight_ry", ry);
    if(ht.has("bspline_dimweight_rz")) ht("bspline_dimweight_rz", rz);
    if(ht.has("bspline_dimweight_tx")) ht("bspline_dimweight_tx", tx);
    if(ht.has("bspline_dimweight_ty")) ht("bspline_dimweight_ty", ty);
    if(ht.has("bspline_dimweight_tz")) ht("bspline_dimweight_tz", tz);
    trajectory.dimweights << rx, ry, rz, tx, ty, tz;

    if(ht.has("bspline_duration")) ht("bspline_duration", trajectory.duration);
    if(ht.has("bspline_task_stiffness")) ht("bspline_task_stiffness", trajectory.stiffness);
    if(ht.has("bspline_task_damping")) ht("bspline_task_damping", trajectory.damping);
    else trajectory.damping = 2.0 * std::sqrt(trajectory.stiffness);

    if(ht.has("bspline_task_weight")) ht("bspline_task_weight", trajectory.weight);
    if(ht.has("bspline_waypoint_height")) ht("bspline_waypoint_height", trajectory.waypoint_height);

    if(ht.has("curve_constraints"))
    {
      auto cc = ht("curve_constraints");
      if(cc.has("final_velocity")) cc("final_velocity", trajectory.final_velocity);
      else if(cc.has("magic_final_velocity")) cc("magic_final_velocity", trajectory.final_velocity);

      if(cc.has("init_velocity")) cc("init_velocity", trajectory.init_velocity);
      else if(cc.has("magic_init_velocity")) cc("magic_init_velocity", trajectory.init_velocity);
    }
  }
}

void HammeringTask_Torque::load_parameters()
{
  params_.load(config_);
}

void HammeringTask_Torque::apply_parameters()
{
  auto postureTask = getPostureTask(robot().name());
  if(postureTask)
  {
    postureTask->stiffness(params_.posture.base_stiffness);
    postureTask->weight(params_.posture.base_weight);
    postureTask->damping(params_.posture.base_damping);
  }

  if(stabilizerTask)
  {
    stabilizerTask->torsoStiffness(params_.stabilizer.torso_stiffness);
    stabilizerTask->torsoWeight(params_.stabilizer.torso_weight);
    stabilizerTask->torsoPitch(params_.stabilizer.torso_pitch);

    stabilizerTask->pelvisStiffness(params_.stabilizer.pelvis_stiffness);
    stabilizerTask->pelvisWeight(params_.stabilizer.pelvis_weight);

    stabilizerTask->dcmGains(params_.stabilizer.dcm_p, params_.stabilizer.dcm_i, params_.stabilizer.dcm_d);
    stabilizerTask->comStiffness(params_.stabilizer.com_stiffness);
    stabilizerTask->comWeight(params_.stabilizer.com_weight);

    for(auto t : solver().tasks())
    {
      if(t->name() == stabilizerTask->name() + "_Tasks_torso")
      {
        auto ot = dynamic_cast<mc_tasks::OrientationTask*>(t);
        if(ot) ot->damping(params_.stabilizer.torso_damping);
      }
      else if(t->name() == stabilizerTask->name() + "_Tasks_pelvis")
      {
        auto ot = dynamic_cast<mc_tasks::OrientationTask*>(t);
        if(ot) ot->damping(params_.stabilizer.pelvis_damping);
      }
      else if(t->name() == stabilizerTask->name() + "_Tasks_com")
      {
        auto ct = dynamic_cast<mc_tasks::CoMTask*>(t);
        if(ct) ct->damping(params_.stabilizer.com_damping);
      }
    }

    stabilizerTask->contactWeight(params_.stabilizer.contact_weight);
    stabilizerTask->contactStiffness(params_.stabilizer.contact_stiffness);
    stabilizerTask->contactDamping(params_.stabilizer.contact_damping);
    stabilizerTask->copAdmittance(params_.stabilizer.contact_admittance);

    auto c = stabilizerTask->config();
    c.dfAdmittance = params_.stabilizer.df_admittance;
    c.dfDamping = params_.stabilizer.df_damping;
    c.torsoPitch = params_.stabilizer.torso_pitch;
    c.comDimWeight = params_.stabilizer.com_dim_weight;
    if(params_.stabilizer.has_com_height)
    {
      c.comHeight = params_.stabilizer.com_height;
    }
    stabilizerTask->configure(c);

    if(params_.stabilizer.has_com_height)
    {
      Eigen::Vector3d com = stabilizerTask->targetCoM();
      com.z() = params_.stabilizer.com_height;
      stabilizerTask->staticTarget(com);
      mc_rtc::log::info("[HammeringTask_Torque] Applied CoM height target: {}", params_.stabilizer.com_height);
    }
  }
}
