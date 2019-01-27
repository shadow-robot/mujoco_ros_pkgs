/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   robot_hw_sim.cpp
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#include <mujoco_ros_control/robot_hw_sim.h>
#include <urdf/model.h>
#include <string>
#include <algorithm>
#include <vector>
#include <limits>
#include <utility>

namespace
{

double clamp(const double val, const double min_val, const double max_val)
{
  return std::min(std::max(val, min_val), max_val);
}

}

namespace mujoco_ros_control
{

RobotHWSim::RobotHWSim()
{
}

bool RobotHWSim::init_sim(
    const std::string& robot_namespace,
    ros::NodeHandle robot_nh,
    mjModel* mujoco_model, mjData* mujoco_data,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions_info,
    int objects_in_scene)
{
  // save references
  mujoco_model_ = mujoco_model;
  mujoco_data_ = mujoco_data;

  const ros::NodeHandle joint_limit_nh(robot_nh);

  // resize vectors to number of DOF
  n_dof_ = mujoco_model_->njnt - objects_in_scene;
  ROS_INFO("%i robot DsOF found.", n_dof_);
  ROS_INFO("%i generalized coordinates (qpos) found.", mujoco_model_->nq);
  ROS_INFO("%i degrees of freedom (qvel) found.", mujoco_model_->nv);
  ROS_INFO("%i actuators/controls (ctrl) found.", mujoco_model_->nu);
  ROS_INFO("%i actuation states (act) found.", mujoco_model_->na);
  ROS_INFO("%i joints (njnt) found.", mujoco_model_->njnt);
  for (int mujoco_joint_id = 0; mujoco_joint_id < n_dof_; mujoco_joint_id++)
  {
    std::string joint_name = mj_id2name(mujoco_model_, mjOBJ_JOINT, mujoco_joint_id);
    MujocoJointData mj_joint_data;
    mj_joint_data.id = mujoco_joint_id;
    mj_joint_data.qpos_addr = mujoco_model_->jnt_qposadr[mujoco_joint_id];
    mj_joint_data.qvel_addr = mujoco_model_->jnt_dofadr[mujoco_joint_id];
    mj_joint_data.type = mujoco_model_->jnt_type[mujoco_joint_id];
    mujoco_joints_.insert(std::pair<std::string, MujocoJointData>(joint_name, mj_joint_data));
  }
  for (auto& mujoco_joint : mujoco_joints_)
  {
    ROS_INFO("%s: %s", mujoco_joint.first.c_str(), mujoco_joint.second.to_string().c_str());
  }
  for (int mujoco_actuator_id = 0; mujoco_actuator_id < mujoco_model_->nu; mujoco_actuator_id++)
  {
    std::string actuator_name = mj_id2name(mujoco_model_, mjOBJ_ACTUATOR, mujoco_actuator_id);
    MujocoActuatorData mj_actuator_data;
    mj_actuator_data.id = mujoco_actuator_id;
    mujoco_actuators_.insert(std::pair<std::string, MujocoActuatorData>(actuator_name, mj_actuator_data));
  }
  for (auto& mujoco_actuator : mujoco_actuators_)
  {
    ROS_INFO("%s: %s", mujoco_actuator.first.c_str(), mujoco_actuator.second.to_string().c_str());
  }
  for (auto& transmission_info : transmissions_info)
  {
    transmissions_.push_back(TransmissionData());
    TransmissionData& transmission = transmissions_.back();
    transmission.name = transmission_info.name_;
    for (auto& joint_info : transmission_info.joints_)
    {
      joints_.insert(std::pair<std::string, JointData>(joint_info.name_, JointData()));
      JointData& joint = joints_.at(joint_info.name_);
      joint.name = joint_info.name_;
      transmission.joint_names.push_back(joint.name);
      joint.mujoco_joint_id = mj_name2id(mujoco_model_, mjOBJ_JOINT, joint.name.c_str());
      ROS_WARN_ONCE("%i actuators/controls (ctrl) found.", mujoco_model_->nu);
      ROS_WARN_ONCE("%i actuation states (act) found.", mujoco_model_->na);
      joint.mujoco_qpos_addr = mujoco_model_->jnt_qposadr[joint.mujoco_joint_id];
      joint.mujoco_qvel_addr = mujoco_model_->jnt_dofadr[joint.mujoco_joint_id];
      if (joint.mujoco_joint_id == -1)
      {
        ROS_WARN("Joint %s not found in Mujoco model!", joint.name.c_str());
      }
      joint.position = 1.0;
      joint.velocity = 0.0;
      joint.effort = 1.0;  // N/m for continuous joints
      joint.effort_command = 0.0;
      joint.position_command = 0.0;
      joint.velocity_command = 0.0;
      joint.hardware_interfaces = joint_info.hardware_interfaces_;

      if (joint.hardware_interfaces.empty())
      {
        ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << joint.name <<
          " of transmission " << transmission_info.name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
      }
      else if (joint.hardware_interfaces.size() > 1)
      {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << joint_info.name_ <<
        " of transmission " << transmission_info.name_ << " specifies multiple hardware interfaces. " <<
        "Currently the robot hardware simulation interface only supports one. Using the first entry");
      }

      if (joint.hardware_interfaces.front() == "EffortJointInterface" ||
          joint.hardware_interfaces.front() == "PositionJointInterface" ||
          joint.hardware_interfaces.front() == "VelocityJointInterface")
      {
        ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" <<
                        joint.hardware_interfaces.front() << "' within the <hardwareInterface> tag in joint '" <<
                        joint.name << "'.");
        joint.hardware_interfaces.front().insert(0, "hardware_interface/");
      }
      if (!transmission_info.actuators_[0].hardware_interfaces_.empty())
      {
        ROS_WARN_STREAM_NAMED("robot_hw_sim", "The <hardware_interface> element of tranmission " <<
          transmission.name << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will not be loaded.");
      }
    }
  }
  for (auto& transmission : transmissions_)
  {
    for (auto& joint_name : transmission.joint_names)
    {
      JointData& joint = joints_.at(joint_name);

      // Debug
      ROS_DEBUG_STREAM_NAMED("robot_hw_sim", "Loading joint '" << joint.name << "' of type '" <<
                             joint.hardware_interfaces.front() << "'.");

      // Create joint state interface for all joints
      ROS_INFO("Registered joint %s with position address %p.", joint.name.c_str(), &joint.position);
      js_interface_.registerHandle(hardware_interface::JointStateHandle(
          joint.name, &joint.position, &joint.velocity, &joint.effort));

      // Decide what kind of command interface this actuator/joint has
      hardware_interface::JointHandle joint_handle;
      if (joint.hardware_interfaces.front() == "hardware_interface/EffortJointInterface")
      {
        // Create effort joint interface
        joint.control_method = EFFORT;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.effort_command);
        ej_interface_.registerHandle(joint_handle);
      }
      else if (joint.hardware_interfaces.front() == "hardware_interface/PositionJointInterface")
      {
        // Create position joint interface
        joint.control_method = POSITION;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.position_command);
        pj_interface_.registerHandle(joint_handle);
      }
      else if (joint.hardware_interfaces.front() == "hardware_interface/VelocityJointInterface")
      {
        // Create velocity joint interface
        joint.control_method = VELOCITY;
        joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint.name),
                                                      &joint.velocity_command);
        vj_interface_.registerHandle(joint_handle);
      }
      else
      {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
          << joint.hardware_interfaces.front() << "' while loading interfaces for " << joint.name);
        return false;
      }

      register_joint_limits(joint.name, joint_handle, joint.control_method,
                            joint_limit_nh, urdf_model,
                            &joint.type, &joint.lower_limit, &joint.upper_limit,
                            &joint.effort_limit);

      if (joint.control_method != EFFORT)
      {
        // Initialize the PID controller. If no PID gain values are found
        const ros::NodeHandle nh(robot_nh, "/mujoco_ros_control/pid_gains/" +
                                joint.name);
        if (joint.pid_controller.init(nh, true))
        {
          switch (joint.control_method)
          {
            case POSITION:
              joint.control_method = POSITION_PID;
              break;
            case VELOCITY:
              joint.control_method = VELOCITY_PID;
              break;
          }
        }
        else
        {
          mujoco_data_->ctrl[joint.mujoco_qvel_addr] = joint.effort_limit;
        }
      }
    }
    if (transmission.joint_names.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Transmission " << transmission.name
        << " has no associated joints.");
    }
    ROS_INFO("%s", transmission.to_string().c_str());
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  return true;
}

void RobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  for (auto& joint_item : joints_)
  {
    JointData& joint = joint_item.second;
    if (string_ends_with(joint.name, "FJ1") || string_ends_with(joint.name, "FJ2"))
    {
      std::string actuator_name = joint.name;
      actuator_name[actuator_name.size() - 1] = '0';
      MujocoActuatorData& actuator = mujoco_actuators_.at(actuator_name);
      joint.effort = mujoco_data_->qfrc_actuator[actuator.id]/2;
    }
    else
    {
      joint.effort = mujoco_data_->qfrc_applied[joint.mujoco_qvel_addr];
    }
    if (joint.type == urdf::Joint::PRISMATIC)
    {
      joint.position = mujoco_data_->qpos[joint.mujoco_qpos_addr];
    }
    else
    {
      joint.position += angles::shortest_angular_distance(joint.position, mujoco_data_->qpos[joint.mujoco_qpos_addr]);
    }
    joint.velocity = mujoco_data_->qvel[joint.mujoco_qvel_addr];
  }
}

void RobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // write the control signals to mujoco data
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  for (auto& actuator : mujoco_actuators_)
  {
    if (string_ends_with(actuator.first, "FJ0"))
    {
      std::string joint_1_name = actuator.first;
      std::string joint_2_name = actuator.first;
      joint_1_name[joint_1_name.size() - 1] = '1';
      joint_2_name[joint_2_name.size() - 1] = '2';
      JointData& joint_1 = joints_.at(joint_1_name);
      JointData& joint_2 = joints_.at(joint_2_name);
      switch (joint_1.control_method)
      {
        case EFFORT:
        {
          mujoco_data_->ctrl[actuator.second.id] = joint_1.effort_command + joint_2.effort_command;;
          ROS_INFO_THROTTLE(1, "Received effort commands %lf and %lf for joints %s and %s respectively; "
                            "set tendon %s effort to %lf.",
                            joint_1.effort_command, joint_2.effort_command, joint_1.name.c_str(), joint_2.name.c_str(),
                            actuator.first.c_str(), mujoco_data_->ctrl[actuator.second.id]);
          break;
        }

        case POSITION:
          mujoco_data_->ctrl[actuator.second.id] = joint_1.position_command + joint_2.position_command;
          break;

        case POSITION_PID:
        {
          mujoco_data_->ctrl[actuator.second.id] =
            clamp(joint_1.pid_controller.computeCommand(joint_1.position_command - joint_1.position, period),
                  -joint_1.effort_limit, joint_1.effort_limit) +
            clamp(joint_2.pid_controller.computeCommand(joint_2.position_command - joint_2.position, period),
                  -joint_2.effort_limit, joint_2.effort_limit);
          break;
        }

        case VELOCITY:
          mujoco_data_->ctrl[actuator.second.id] = joint_1.velocity_command + joint_2.velocity_command;
          break;

        case VELOCITY_PID:
          mujoco_data_->ctrl[actuator.second.id] =
            clamp(joint_1.pid_controller.computeCommand(joint_1.velocity_command - joint_1.velocity, period),
                  -joint_1.effort_limit, joint_1.effort_limit) +
            clamp(joint_2.pid_controller.computeCommand(joint_2.velocity_command - joint_2.velocity, period),
                  -joint_2.effort_limit, joint_2.effort_limit);
          break;
      }
      continue;
    }
    for (auto& joint_item : joints_)
    {
      JointData& joint = joint_item.second;
      if ( actuator.first == joint.name)
      {
        switch (joint.control_method)
        {
          case EFFORT:
          {
            mujoco_data_->ctrl[actuator.second.id] = joint.effort_command;
          }
          break;

          case POSITION:
          {
            mujoco_data_->ctrl[actuator.second.id] = joint.position_command;
          }
          break;

          case POSITION_PID:
          {
            double error;

            error = joint.position_command - joint.position;
            const double effort_limit = joint.effort_limit;
            const double effort = clamp(joint.pid_controller.computeCommand(error, period),
                                        -effort_limit, effort_limit);
            mujoco_data_->ctrl[actuator.second.id] = effort;
          }
          break;

          case VELOCITY:
            mujoco_data_->ctrl[actuator.second.id] = joint.velocity_command;
            break;

          case VELOCITY_PID:
            double error;
            error = joint.velocity_command - joint.velocity;
            const double effort_limit = joint.effort_limit;
            const double effort = clamp(joint.pid_controller.computeCommand(error, period),
                                        -effort_limit, effort_limit);
            mujoco_data_->ctrl[actuator.second.id] = effort;
            break;
        }
        continue;
      }
    }
  }
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void RobotHWSim::register_joint_limits(const std::string& joint_name,
                           const hardware_interface::JointHandle& joint_handle,
                           const ControlMethod ctrl_method,
                           const ros::NodeHandle& joint_limit_nh,
                           const urdf::Model *const urdf_model,
                           int *const joint_type, double *const lower_limit,
                           double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}

bool RobotHWSim::string_ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

}  // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::RobotHWSim, mujoco_ros_control::RobotHWSimPlugin)
