/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_ros_control.h
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
 **/

#ifndef MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
#define MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>

// Mujoco dependencies
#include <mujoco.h>
#include <mjdata.h>
#include <mjmodel.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>

// ros_control
#include <mujoco_ros_control/robot_hw_sim.h>
#include <mujoco_ros_control/robot_hw_sim_plugin.h>

// msgs
#include "geometry_msgs/Pose.h"
#include "mujoco_ros_msgs/FreeObjectsStates.h"

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// openGL stuff
#include <glfw3.h>
#include <mujoco_ros_control/visualization_utils.h>

#include <rosgraph_msgs/Clock.h>

namespace mujoco_ros_control
{

class MujocoRosControl
{
public:
  MujocoRosControl();
  virtual ~MujocoRosControl();

  // initialize params and controller manager
  void init(ros::NodeHandle &nodehandle);

  // step update function
  void update();

  unsigned int n_dof_;
  std::vector<int> objects_in_scene_;

  // pointer to the mujoco model
  mjModel* mujoco_model;
  mjData* mujoco_data;

protected:
  // get the URDF XML from the parameter server
  std::string get_urdf(std::string param_name) const;

  // setup initial sim environment
  void setup_sim_environment(const float initial_pos[]);

  // parse transmissions from URDF
  bool parse_transmissions(const std::string& urdf_string);

  // publish simulation time to ros clock
  void publish_sim_time();

  // check for free joints in the mujoco model
  void check_objects_in_scene();

  // publish free objects
  void publish_objects_in_scene();

  // node handles
  ros::NodeHandle robot_node_handle;

  // interface loader
  boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin> > robot_hw_sim_loader_;

  // strings
  std::string robot_namespace_;
  std::string robot_description_param_;
  std::string robot_model_path_;

  std::vector<int> mujoco_ids;
  std::vector<int>::iterator it;

  // transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // robot simulator interface
  boost::shared_ptr<mujoco_ros_control::RobotHWSimPlugin> robot_hw_sim_;

  // controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // simulated clock
  ros::Publisher pub_clock_;
  int pub_clock_frequency_;
  ros::Time last_pub_clock_time_;

  // timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  // publishing
  ros::Publisher objects_in_scene_publisher = robot_node_handle.advertise<mujoco_ros_msgs::FreeObjectsStates>
                                                                         ("/mujoco/free_objects_states", 1000);
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
