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
  ros::NodeHandle model_nh,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // resize all vectors based on model joints

  // create joint interfaces

  // register interfaces

  return true;
}

void RobotHWSim::read(const ros::Time& time, const ros::Duration& period)
{
  // fill up joint_positions vector with current position to update joint state controller
}

void RobotHWSim::write(const ros::Time& time, const ros::Duration& period)
{
  // write the control signals d->ctrl (how to pass the d? -> we may need re-implementation)
}

}

PLUGINLIB_EXPORT_CLASS( mujoco_ros_control::RobotHWSim, hardware_interface::RobotHW)
