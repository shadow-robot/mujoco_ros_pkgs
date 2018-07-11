/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_ros_control.cpp
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/


#include <boost/bind.hpp>
#include <mujoco_ros_control/mujoco_ros_control.h>
#include <mujoco_ros_control/visualization_utils.h>
#include <urdf/model.h>
#include <string>
#include <vector>

namespace mujoco_ros_control
{
MujocoRosControl::MujocoRosControl()
: objects_in_scene(0), n_dof_(0)
{}

MujocoRosControl::~MujocoRosControl()
{
  // deallocate existing mjModel
  mj_deleteModel(mujoco_model);

  // deallocate existing mjData
  mj_deleteData(mujoco_data);

  mj_deactivate();
}

void MujocoRosControl::init(ros::NodeHandle &nodehandle)
{
      // Check that ROS has been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Unable to initialize Mujoco node.");
        return;
    }

    // activation license mujoco
    mj_activate("/home/user/mjpro150/bin/mjkey.txt");

    // publish clock for simulated time
    pub_clock_ = nodehandle.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // create robot node handle
    robot_node_handle = ros::NodeHandle("/");

    ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control node in namespace: %s", robot_namespace_.c_str());

    // read urdf from ros parameter server then setup actuators and mechanism control node.
    if (nodehandle.getParam("mujoco_ros_control/robot_description_param", robot_description_param_))
    {
      ROS_INFO("Got param Robot description: %s", robot_description_param_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_description_param'");
    }

    const std::string urdf_string = get_urdf(robot_description_param_);

    if (!parse_transmissions(urdf_string))
    {
      ROS_ERROR_NAMED("mujoco_ros_control", "Error parsing URDF in mujoco_ros_control node, node not active.\n");
      return;
    }

    if (nodehandle.getParam("mujoco_ros_control/robot_model_path", robot_model_path_))
    {
      ROS_INFO("Got param: %s", robot_model_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_model_path'");
    }

    char error[1000];

    // create mjModel
    mujoco_model = mj_loadXML(robot_model_path_.c_str(), NULL, error, 1000);
    if (!mujoco_model)
    {
      printf("Could not load mujoco model with error: %s.\n", error);
      return;
    }

    // create mjData corresponding to mjModel
    mujoco_data = mj_makeData(mujoco_model);
    if (!mujoco_data)
    {
      printf("Could not create mujoco data from model.\n");
      return;
    }

    // check for free joints
    check_objects_in_scene();

    // get the Mujoco simulation period
    ros::Duration mujoco_period(mujoco_model->opt.timestep);

    // set control period as mujoco_period
    control_period_ = mujoco_period;

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin>
          ("mujoco_ros_control", "mujoco_ros_control::RobotHWSimPlugin"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHWSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if (!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, mujoco_model,
                                 mujoco_data, urdf_model_ptr, transmissions_, objects_in_scene))
    {
      ROS_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot sim interface");
      return;
    }

    // create the controller manager
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_node_handle));
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("mujoco_ros_control" , "Failed to create robot sim interface loader: "
                             << ex.what());
    }
    ROS_INFO_NAMED("mujoco_ros_control", "Loaded mujoco_ros_control.");

    mj_resetData(mujoco_model, mujoco_data);

    contact_force = mj_stackAlloc(mujoco_data, 6);
}

void MujocoRosControl::update()
{
  publish_sim_time();

  ros::Time sim_time = (ros::Time)mujoco_data->time;
  ros::Time sim_time_ros(sim_time.sec, sim_time.nsec);

  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  mj_step1(mujoco_model, mujoco_data);

  mujoco_contacts = mujoco_data->ncon;

  for (int i = 0; i < mujoco_contacts; i++)
  {
    mj_contactForce(mujoco_model, mujoco_data, i, contact_force);
    // std::cout << "Contact force for contact    " << i << '\n';
    // mju_printMat(contact_force, 6, 1);
  }

  // check if we should update the controllers
  if (sim_period >= control_period_)
  {
    // store simulation time
    last_update_sim_time_ros_ = sim_time_ros;

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

// get the URDF XML from the parameter server
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
        " URDF in parameter [%s] on the ROS param server.", robot_description_param_.c_str());

      robot_node_handle.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Received urdf from param server, parsing...");

  return urdf_string;
}

// get Transmissions from the URDF
bool MujocoRosControl::parse_transmissions(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

void MujocoRosControl::publish_sim_time()
{
  ros::Time sim_time = (ros::Time)mujoco_data->time;
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).toSec() < 1.0/pub_clock_frequency_)
    return;

  ros::Time current_time = (ros::Time)mujoco_data->time;
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(current_time.toSec());
  //  publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

int MujocoRosControl::check_objects_in_scene()
{
  n_dof_ = mujoco_model->njnt;

  for (int i=0; i < n_dof_; i++)
  {
    int *number_of_free_joint = &(mujoco_model->jnt_type[i]);
    int jnt_type = *number_of_free_joint;
    if (jnt_type == 0)
    {
      ROS_INFO("Free Joint Found");
      objects_in_scene += 1;
    }
  }
  return objects_in_scene;
}
}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujoco_ros_control");

    ros::NodeHandle nh_;

    mujoco_ros_control::MujocoRosControl mujoco_ros_control;

    mujoco_ros_control::MujocoVisualizationUtils &mujoco_visualization_utils =
        mujoco_ros_control::MujocoVisualizationUtils::getInstance();

    // initialize mujoco stuff
    mujoco_ros_control.init(nh_);

    // init GLFW
    if ( !glfwInit() )
      mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // make context current
    glfwMakeContextCurrent(window);

    // initialize mujoco visualization functions
    mujoco_visualization_utils.init(mujoco_ros_control.mujoco_model, mujoco_ros_control.mujoco_data, window);

    // spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // // let everything settle
    std::vector<double> initial_qpos;
    initial_qpos.assign(mujoco_ros_control.n_dof_, 0);

    while (mujoco_ros_control.mujoco_data->time < 30)
    {
      mj_step1(mujoco_ros_control.mujoco_model, mujoco_ros_control.mujoco_data);
      for (int i=0; i < mujoco_ros_control.n_dof_-mujoco_ros_control.objects_in_scene; i++)
      {
        mujoco_ros_control.mujoco_data->ctrl[i] = initial_qpos[i] + mujoco_ros_control.mujoco_data->qfrc_bias[i];
      }
      mj_step2(mujoco_ros_control.mujoco_model, mujoco_ros_control.mujoco_data);
    }
    mju_copy(mujoco_ros_control.mujoco_data->qpos, mujoco_ros_control.mujoco_model->key_qpos,
             mujoco_ros_control.mujoco_model->nq*1);

    // run main loop, target real-time simulation and 60 fps rendering
    while ( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        mjtNum sim_start = mujoco_ros_control.mujoco_data->time;

        while ( mujoco_ros_control.mujoco_data->time - sim_start < 1.0/60.0 && ros::ok() )
        {
          mujoco_ros_control.update();
        }
        mujoco_visualization_utils.update(window);
    }

    mujoco_visualization_utils.terminate();

    return 0;
}
