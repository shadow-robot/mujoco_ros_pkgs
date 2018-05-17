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
//#include "mujoco.h"
//#include "mjdata.h"
//#include "mjmodel.h"

// ros_control
#include <mujoco_ros_control/robot_hw_sim.h>
#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

namespace mujoco_ros_control
{

class MujocoRosControl
{
public:

  virtual ~MujocoRosControl();

  // initialize params and controller manager
  void init();

  // step update function
  void update(const ros::Time& time, const ros::Duration& period);


protected:

  // node handles
  ros::NodeHandle robot_node_handle; //namespaces to the robot name

  // pointer to the mujoco model
  //mjModel* m;
  //mjData* d;

  // interface loader
  boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim> > robot_hw_sim_loader_;

  // Strings
  std::string robot_namespace_;
  std::string robot_description_;

  // Robot simulator interface
  std::string robot_hw_sim_type_str_;
  boost::shared_ptr<mujoco_ros_control::RobotHWSim> robot_hw_sim_;

  // Controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

};
}
#endif // #ifndef __MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H_
