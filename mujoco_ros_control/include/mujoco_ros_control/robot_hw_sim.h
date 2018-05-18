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
#include "/home/user/mjpro150/include/mujoco.h"
#include "/home/user/mjpro150/include/mjdata.h"
#include "/home/user/mjpro150/include/mjmodel.h"

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
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
    mjModel* mujoco_model, mjData *mujoco_data,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void read(const ros::Time& time, const ros::Duration& period);

  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  // Register the limits of the joint specified by joint_name and joint_handle. The limits are
  // retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  void register_joint_limits(const std::string& joint_name,
                             const hardware_interface::JointHandle& joint_handle,
                             const ControlMethod ctrl_method,
                             const ros::NodeHandle& joint_limit_nh,
                             const urdf::Model *const urdf_model,
                             int *const joint_type, double *const lower_limit,
                             double *const upper_limit, double *const effort_limit);

  unsigned int n_dof_;

  // hardware interfaces
  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::EffortJointInterface   ej_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  hardware_interface::VelocityJointInterface vj_interface_;

  // joint limit interfaces
  joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;
  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;

  // vectors
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<ControlMethod> joint_control_methods_;
  std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> last_joint_position_command_;
  std::vector<double> joint_velocity_command_;

  //mujoco elements
  mjModel* mujoco_model_;
  mjData* mujoco_data_;

};

typedef boost::shared_ptr<RobotHWSim> RobotHWSimPtr;

}

#endif // #ifndef __MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_H_
