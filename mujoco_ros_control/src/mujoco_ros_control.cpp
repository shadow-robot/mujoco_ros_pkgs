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
#include <string>

namespace mujoco_ros_control
{

MujocoRosControl::~MujocoRosControl()
{
  // deallocate existing mjModel
  mj_deleteModel(mujoco_model);

  // deallocate existing mjData
  mj_deleteData(mujoco_data);

  mj_deactivate();
}

void MujocoRosControl::init()
{
      // Check that ROS has been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Unable to initialize Mujoco node.");
        return;
    }

    // activation license mujoco
    mj_activate("/home/user/mjpro150/bin/mjkey.txt");

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

    // get package path and filename
    std::string package_path = ros::package::getPath("mujoco_ros_control");
    std::string name_file = "/robot_model.xml";
    std::string filename = package_path + name_file;

    // write xml to file
    std::ofstream out(filename.c_str());
    out << urdf_string;
    out.close();

    char error[1000];

    // create mjModel
    mujoco_model = mj_loadXML(filename.c_str(), NULL, error, 1000);
    if (!mujoco_model)
    {
      printf("Could not load mujoco model.\n");
      return;
    }

    // create mjData corresponding to mjModel
    mujoco_data = mj_makeData(mujoco_model);
    if (!mujoco_data)
    {
      printf("Could not create mujoco data from model.\n");
      return;
    }
    // get the Mujoco simulation period
    ros::Duration mujoco_period(mujoco_model->opt.timestep);

    // set control period as mujoco_period
    control_period_ = mujoco_period;

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSim>
          ("mujoco_ros_control", "mujoco_ros_control::RobotHWSim"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHwSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    if (!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, mujoco_model,
                                 mujoco_data, urdf_model_ptr, transmissions_))
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
}

void MujocoRosControl::update()
{
  // get simulation time and period
  int64_t nanosec_time = (mujoco_data->time) * 1e9;
  ros::Time sim_time_ros(mujoco_data->time, nanosec_time);

  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // check if we should update the controllers
  if (sim_period >= control_period_) {
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
        " URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

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

}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujoco_ros_control");

    mujoco_ros_control::MujocoRosControl MujocoRosControl;

    // initialize mujoco stuff
    MujocoRosControl.init();

    // MuJoCo visualization
    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;

    // init GLFW
    if ( !glfwInit() )
      mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // make context current
    glfwMakeContextCurrent(window);

    // initialize MuJoCo visualization
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjr_makeContext(MujocoRosControl.mujoco_model, &con, 200);

    // center and scale view
    cam.lookat[0] = MujocoRosControl.mujoco_model->stat.center[0];
    cam.lookat[1] = MujocoRosControl.mujoco_model->stat.center[1];
    cam.lookat[2] = MujocoRosControl.mujoco_model->stat.center[2];
    cam.distance = 1.5 * MujocoRosControl.mujoco_model->stat.extent;

    // run main loop, target real-time simulation and 60 fps rendering
    while ( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        mjtNum sim_start = MujocoRosControl.mujoco_data->time;
        while ( MujocoRosControl.mujoco_data->time - sim_start < 1.0/60.0 && ros::ok() )
        {
            MujocoRosControl.update();
        }
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(MujocoRosControl.mujoco_model,  MujocoRosControl.mujoco_data, &opt,
                        NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // free scene and terminate glfw
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();

    ros::spin();
    return 0;
}
