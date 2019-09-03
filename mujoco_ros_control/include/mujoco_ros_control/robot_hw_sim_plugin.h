/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   mujoco_ros_control.h
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
**/

#ifndef MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_PLUGIN_H
#define MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_PLUGIN_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

// Mujoco dependencies
#include <mujoco.h>
#include <mjdata.h>
#include <mjmodel.h>

#include <string>
#include <vector>


namespace mujoco_ros_control
{
// Struct for passing loaded joint data
struct JointData
{
  std::string name_;
  std::string hardware_interface_;

  JointData(const std::string& name, const std::string& hardware_interface) :
    name_(name),
    hardware_interface_(hardware_interface)
  {}
};

class RobotHWSimPlugin : public hardware_interface::RobotHW
{
public:
  virtual ~RobotHWSimPlugin() { }

  virtual bool init_sim(
      const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      mjModel* mujoco_model, mjData *mujoco_data,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions,
      int objects_in_scene) = 0;
};
}  // namespace mujoco_ros_control

#endif  // MUJOCO_ROS_CONTROL_ROBOT_HW_SIM_PLUGIN_H
