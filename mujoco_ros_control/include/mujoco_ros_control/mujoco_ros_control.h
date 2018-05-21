/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_ros_control.h
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
 **/

#ifndef _MUJOCO_ROS_CONTROL___MUJOCO_ROS_CONTROL_H_
#define _MUJOCO_ROS_CONTROL___MUJOCO_ROS_CONTROL_H_

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

// Mujoco dependencies
#include "/home/user/mjpro150/include/mujoco.h"
#include "/home/user/mjpro150/include/mjdata.h"
#include "/home/user/mjpro150/include/mjmodel.h"

// ros_control
#include <mujoco_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// openGL stuff
#include <GLFW/glfw3.h>

namespace mujoco_ros_control
{

class MujocoRosControl
{
public:

  virtual ~MujocoRosControl();

  // initialize params and controller manager
  void init();

  // step update function
  void update();

  // initialize glfw function
  void init_glfw();

  // get the URDF XML from the parameter server
  std::string get_urdf(std::string param_name) const;

  // parse transmissions from URDF
  bool parse_transmissions(const std::string& urdf_string);

  // pointer to the mujoco model
  mjModel* mujoco_model;
  mjData* mujoco_data;

protected:

  // node handles
  ros::NodeHandle robot_node_handle; //namespaces to the robot name

  // interface loader
  boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim> > robot_hw_sim_loader_;

  // strings
  std::string robot_namespace_;
  std::string robot_description_;

  // transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // robot simulator interface
  boost::shared_ptr<mujoco_ros_control::RobotHWSim> robot_hw_sim_;

  // controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

};
}
#endif // #ifndef __MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H_
