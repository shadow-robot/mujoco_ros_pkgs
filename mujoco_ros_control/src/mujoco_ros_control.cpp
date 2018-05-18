/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_ros_control.cpp
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/


#include <boost/bind.hpp>
#include <mujoco_ros_control/mujoco_ros_control.h>
#include <urdf/model.h>

namespace mujoco_ros_control
{

MujocoRosControl::~MujocoRosControl()
{
  // disconnect from mujoco events

  // deallocate existing mjModel
  //mj_deleteModel(m);

  // deallocate existing mjData
  //mj_deleteData(d);
}

void MujocoRosControl::init()
{

      // Check that ROS has been initialized
    if(!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control","A ROS node for Mujoco has not been initialized, unable to initialize node. ");
        return;
    }
    

    // activation license mujoco
    //mj_activate(".txt")

    // create mjModel
    mujoco_model = mj_loadModel("/home/user/projects/shadow_robot/base/src/mujoco_ros_pkgs/mujoco_ros_control/config/test_urdf.xml", NULL);

    // create mjData corresponding to mjModel
    mujoco_data = mj_makeData(mujoco_model);


    // get the Mujoco simulation period
    ros::Duration mujoco_period(mujoco_model->opt.timestep);

    // set control period as mujoco_period
    control_period_ = mujoco_period;
    ROS_DEBUG_STREAM_NAMED("gazebo_ros_control","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
      
    // create robot node handle
    robot_node_handle = ros::NodeHandle("/");

    ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control node in namespace: %s", robot_namespace_.c_str());

    // read urdf from ros parameter server then setup actuators and mechanism control node.
    robot_description_ = "robot_description";

    const std::string urdf_string = get_urdf(robot_description_);

    if (!parse_transmissions(urdf_string))
    {
      ROS_ERROR_NAMED("mujoco_ros_control", "Error parsing URDF in mujoco_ros_control node, node not active.\n");
      return;
    }

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim>
          ("mujoco_ros_control", "mujoco_ros_control::RobotHWSim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHwSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if(!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, mujoco_model, mujoco_data, urdf_model_ptr, transmissions_))
    {
      ROS_FATAL_NAMED("mujoco_ros_control","Could not initialize robot simulation interface");
      return;
    }

    // create the controller manager
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_node_handle));
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("mujoco_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
    }
\
    ROS_INFO_NAMED("mujoco_ros_control", "Loaded mujoco_ros_control.");
}

void MujocoRosControl::update(const ros::Time& time, const ros::Duration& period)
{
  // get simulation time and period
  ros::Time sim_time_ros(mujoco_data->time, mujoco_data->time); 

  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // check if we should update the controllers
  if(sim_period >= control_period_) {
    // store simulation time
    last_update_sim_time_ros_ = sim_time_ros;
    
    // call first step of simulation without control signals
    mj_step1(mujoco_model, mujoco_data);

    // update the robot simulation with the state of the mujoco model
    robot_hw_sim_->read(sim_time_ros, sim_period);

    bool reset_ctrls = false;

    // compute the controller commands
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrls);
  }

  // update the mujoco model with the result of the controller
  robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);
  last_write_sim_time_ros_ = sim_time_ros;
  mj_step2(mujoco_model, mujoco_data);
}

// Get the URDF XML from the parameter server
std::string MujocoRosControl::get_urdf(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (robot_node_handle.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_node_handle.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

      robot_node_handle.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Received urdf from param server, parsing...");

  return urdf_string;
}

// Get Transmissions from the URDF
bool MujocoRosControl::parse_transmissions(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mujoco_ros_control");

    mujoco_ros_control::MujocoRosControl MujocoRosControl();

    MujocoRosControl().init();
    // init openGL window


    ros::spin();
    return 0;
} 

