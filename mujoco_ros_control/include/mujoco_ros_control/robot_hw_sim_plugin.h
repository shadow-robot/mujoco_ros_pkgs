/// \brief Plugin template for hardware interfaces for ros_control and Gazebo

/// \author Jonathan Bohren
/// \author Dave Coleman

#ifndef ROS_CONTROL_MUJOCO_ROBOT_HW_SIM_PLUGIN_H
#define ROS_CONTROL_MUJOCO_ROBOT_HW_SIM_PLUGIN_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

// Mujoco dependencies
#include </home/user/mjpro150/include/mujoco.h>
#include </home/user/mjpro150/include/mjdata.h>
#include </home/user/mjpro150/include/mjmodel.h>


namespace mujoco_ros_control {

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

  /// \brief Mujoco plugin version of RobotHW
  /// An object of class RobotHWSim represents a robot's simulated hardware.
  class RobotHWSimPlugin : public hardware_interface::RobotHW
  {
  public:

    virtual ~RobotHWSimPlugin() { }

    /// \brief Initialize the simulated robot hardware
    ///
    /// Initialize the simulated robot hardware.
    ///
    /// \param robot_namespace  Robot namespace.
    /// \param model_nh  Model node handle.
    /// \param parent_model  Parent model.
    /// \param urdf_model  URDF model.
    /// \param transmissions  Transmissions.
    ///
    /// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
    virtual bool init_sim(
        const std::string& robot_namespace,
        ros::NodeHandle model_nh,
        mjModel* mujoco_model, mjData *mujoco_data,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions) = 0;

  };

}  // namespace mujoco_ros_control

#endif // ifndef ROS_CONTROL_MUJOCO_ROBOT_HW_SIM_PLUGIN_H