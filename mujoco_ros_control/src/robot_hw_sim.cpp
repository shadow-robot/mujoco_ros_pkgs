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
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // save references
  mujoco_model_ = mujoco_model;
  mujoco_data_ = mujoco_data;

  const ros::NodeHandle joint_limit_nh(robot_nh);

  // resize vectors to number of DOF
  n_dof_ = mujoco_model->njnt;
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);

  // Initialize values
  for (unsigned int j = 0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if (transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if (transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Transmission " << transmissions[j].name_
        << " has more than one joint. Currently the default robot hardware simulation "
        << " interface only supports one.");
      continue;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty()))
    {
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the robot hardware simulation interface only supports one. Using the first entry");
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    const std::string& hardware_interface = joint_interfaces.front();

    // Debug
    ROS_DEBUG_STREAM_NAMED("robot_hw_sim", "Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if (hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle);
    }
    else if (hardware_interface == "PositionJointInterface" ||
             hardware_interface == "hardware_interface/PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);
    }
    else if (hardware_interface == "VelocityJointInterface" ||
             hardware_interface == "hardware_interface/VelocityJointInterface")
    {
      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
        << hardware_interface << "' while loading interfaces for " << joint_names_[j]);
      return false;
    }

    if (hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" ||
        hardware_interface == "VelocityJointInterface") {
      ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" <<
                      hardware_interface << "' within the <hardwareInterface> tag in joint '" <<
                      joint_names_[j] << "'.");
    }

  register_joint_limits(joint_names_[j], joint_handle, joint_control_methods_[j],
                        joint_limit_nh, urdf_model,
                        &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                        &joint_effort_limits_[j]);

  if (joint_control_methods_[j] != EFFORT)
  {
    // Initialize the PID controller. If no PID gain values are found
    const ros::NodeHandle nh(robot_nh, "/mujoco_ros_control/pid_gains/" +
                             joint_names_[j]);
    if (pid_controllers_[j].init(nh, true))
    {
      switch (joint_control_methods_[j])
      {
        case POSITION:
          joint_control_methods_[j] = POSITION_PID;
          break;
        case VELOCITY:
          joint_control_methods_[j] = VELOCITY_PID;
          break;
      }
    }
    else
    {
      mujoco_data_->ctrl[j] = joint_effort_limits_[j];
    }
  }
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
  // fill up joint_positions vector with current position to update joint state controller
  // get current state of simulation
  for (unsigned int j = 0; j < n_dof_; j++)
  {
    double position;
    position = mujoco_data_->qpos[j];

    if (joint_types_[j] == urdf::Joint::PRISMATIC)
    {
      joint_position_[j] = position;
    }
    else
    {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                            position);
    }
    joint_velocity_[j] = mujoco_data_->qvel[j];
    joint_effort_[j] = mujoco_data_->qacc[j];

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

  for (unsigned int j = 0; j < n_dof_; j++)
  {
    switch (joint_control_methods_[j])
    {
      case EFFORT:
      {
        mujoco_data_->ctrl[j] = joint_effort_command_[j];
      }
      break;

      case POSITION:
      {
        mujoco_data_->ctrl[j] = joint_position_command_[j];
      }
      break;

      case POSITION_PID:
        {
          double error;

          error = joint_position_command_[j] - joint_position_[j];

          const double effort_limit = joint_effort_limits_[j];
          const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                      -effort_limit, effort_limit);
          mujoco_data_->ctrl[j] = effort;
        }
        break;

      case VELOCITY:
        mujoco_data_->ctrl[j] = joint_velocity_command_[j];
        break;

      case VELOCITY_PID:
        double error;
        error = joint_velocity_command_[j] - joint_velocity_[j];
        const double effort_limit = joint_effort_limits_[j];
        const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                                    -effort_limit, effort_limit);
        mujoco_data_->ctrl[j] = effort;
        break;
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
}  // namespace mujoco_ros_control

PLUGINLIB_EXPORT_CLASS(mujoco_ros_control::RobotHWSim, mujoco_ros_control::RobotHWSimPlugin)
