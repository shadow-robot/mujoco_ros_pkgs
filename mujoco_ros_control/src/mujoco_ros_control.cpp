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

    // get urdf, load robotHwSim and load controller manager

    // activation license mujoco
    // mj_activate(".txt")

    // create mjModel 
    //m = mj_loadModel(".xml")
    
    // create mjData corresponding to mjModel
    //d = mj_makeData(m) 
}

void MujocoRosControl::update(const ros::Time& time, const ros::Duration& period)
{
    // send control signals
    //mj_step1(m, d)
    // call write from RobotHWSim
    //mj_step2(m, d)
}

std::string MujocoRosControl::load_URDF(std::string urdf_file_name) const
{
    // search the URDF file by name and load it in Mujoco
}
} // namespace

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mujoco_ros_control");

    mujoco_ros_control::MujocoRosControl MujocoRosControl();

    ros::spin();
    return 0;
} 

