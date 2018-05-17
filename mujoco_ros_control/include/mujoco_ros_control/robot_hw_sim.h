/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   robot_hw_sim.h
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#ifndef _MUJOCO_ROS_CONTROL___ROBOT_HW_SIM_H_
#define _MUJOCO_ROS_CONTROL___ROBOT_HW_SIM_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_info.h>

// Mujoco dependencies

// ROS
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

namespace mujoco_ros_control
{

class RobotHWSim : public hardware_interface::RobotHW
{
public:
  RobotHWSim();
  virtual ~RobotHWSim(){};

  virtual bool init_sim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    //mjModel &m, //mjData &d,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void read(const ros::Time& time, const ros::Duration& period);

  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:

  // hardware interfaces
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  // vectors
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  //mujoco elements
  //mjModel* m;
  //mjData* d;

};

typedef boost::shared_ptr<RobotHWSim> RobotHWSimPtr;

}

#endif // #ifndef __MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_H_
