/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   robot_hw_sim.h
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#ifndef MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_H
#define MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_H

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <mujoco_ros_control/robot_hw_sim_plugin.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <transmission_interface/transmission_info.h>

// Mujoco dependencies
#include <mujoco.h>
#include <mjdata.h>
#include <mjmodel.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

#include <map>
#include <string>
#include <vector>

namespace mujoco_ros_control
{

class RobotHWSim : public mujoco_ros_control::RobotHWSimPlugin
{
public:
  RobotHWSim();
  virtual ~RobotHWSim() {}

  virtual bool init_sim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    mjModel* mujoco_model, mjData *mujoco_data,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions_info,
    int objects_in_scene);

  virtual void read(const ros::Time& time, const ros::Duration& period);

  virtual void write(const ros::Time& time, const ros::Duration& period);

  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};

  struct JointData
  {
    std::string name;
    int type;
    double lower_limit;
    double upper_limit;
    double effort_limit;
    ControlMethod control_method;
    control_toolbox::Pid pid_controller;
    double position;
    double velocity;
    double effort;
    double effort_command;
    double position_command;
    double last_position_command;
    double velocity_command;
    std::vector<std::string> hardware_interfaces;
    int mujoco_joint_id;
    int mujoco_qpos_addr;
    int mujoco_qvel_addr;

    std::string to_string()
    {
      std::stringstream ss;
      ss << "Joint " << name << " has type " << type << ", mujoco addresses " << mujoco_joint_id << ", " <<
        mujoco_qpos_addr << ", " << mujoco_qvel_addr << ".\nJoint status: p:" << position << " v:" << velocity <<
        " e:" << effort << "\nJoint position address: " << &position;
      return ss.str();
    }
  };

  struct TransmissionData
  {
    std::string name;
    std::vector<std::string> joint_names;
    std::string to_string()
    {
      std::stringstream ss;
      ss << "Transmission " << name << " has " << joint_names.size() << " joints:";
      for (auto& joint_name : joint_names)
      {
        ss << "\n" << joint_name;
      }
      return ss.str();
    }
  };

  struct MujocoJointData
  {
    int id;
    int qpos_addr;
    int qvel_addr;
    int type;

    std::string to_string()
    {
      std::stringstream ss;
      ss << "Mujoco Joint has type " << type << ", mujoco addresses " << id << ", " <<
        qpos_addr << ", " << qvel_addr;
      return ss.str();
    }
  };

  struct MujocoActuatorData
  {
    int id;

    std::string to_string()
    {
      std::stringstream ss;
      ss << "Mujoco Actuator has mujoco address " << id << ".";
      return ss.str();
    }
  };

protected:
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

  static bool string_ends_with(std::string const & value, std::string const & ending);

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

  std::vector<TransmissionData> transmissions_;
  std::map<std::string, JointData> joints_;
  std::map<std::string, MujocoJointData> mujoco_joints_;
  std::map<std::string, MujocoActuatorData> mujoco_actuators_;

  // mujoco elements
  mjModel* mujoco_model_;
  mjData* mujoco_data_;
};
typedef boost::shared_ptr<RobotHWSim> RobotHWSimPtr;
}  // namespace mujoco_ros_control

#endif  // MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_H
