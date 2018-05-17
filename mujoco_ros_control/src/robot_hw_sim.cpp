/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   robot_hw_sim.cpp
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#include <mujoco_ros_control/robot_hw_sim.h>
#include <urdf/model.h>

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
  for(unsigned int j=0; j < n_dof_; j++)
  {
    // Check that this transmission has one joint
    if(transmissions[j].joints_.size() == 0)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
        << " has no associated joints.");
      continue;
    }
    else if(transmissions[j].joints_.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim","Transmission " << transmissions[j].name_
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
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
        transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
        "The transmission will be properly loaded, but please update " <<
        "your robot model to remain compatible with future versions of the plugin.");
    }
    if (joint_interfaces.empty())
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
        "Not adding it to the robot hardware simulation.");
      continue;
    }
    else if (joint_interfaces.size() > 1)
    {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
        " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
        "Currently the default robot hardware simulation interface only supports one. Using the first entry");
      //continue;
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
    ROS_DEBUG_STREAM_NAMED("default_robot_hw_sim","Loading joint '" << joint_names_[j]
      << "' of type '" << hardware_interface << "'");

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    // Decide what kind of command interface this actuator/joint has
    hardware_interface::JointHandle joint_handle;
    if(hardware_interface == "EffortJointInterface" || hardware_interface == "hardware_interface/EffortJointInterface")
    {
      // Create effort joint interface
      joint_control_methods_[j] = EFFORT;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
      ej_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "PositionJointInterface" || hardware_interface == "hardware_interface/PositionJointInterface")
    {
      // Create position joint interface
      joint_control_methods_[j] = POSITION;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
      pj_interface_.registerHandle(joint_handle);
    }
    else if(hardware_interface == "VelocityJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface")
    {
      // Create velocity joint interface
      joint_control_methods_[j] = VELOCITY;
      joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
      vj_interface_.registerHandle(joint_handle);
    }
    else
    {
      ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
        << hardware_interface << "' while loading interfaces for " << joint_names_[j] );
      return false;
    }

    if(hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface") {
      ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface << "' within the <hardwareInterface> tag in joint '" << joint_names_[j] << "'.");
    }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  return true;
  } 
}

void RobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  // fill up joint_positions vector with current position to update joint state controller
  // get current state of simulation
  for(unsigned int j=0; j < n_dof_; j++)
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
  // write the control signals d->ctrl (how to pass the d? -> we may need re-implementation)
}

}

PLUGINLIB_EXPORT_CLASS( mujoco_ros_control::RobotHWSim, hardware_interface::RobotHW)
