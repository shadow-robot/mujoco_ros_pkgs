/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_visualization_utils.h
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#ifndef MUJOCO_ROS_CONTROL_MUJOCO_VISUALIZATION_UTILS_H
#define MUJOCO_ROS_CONTROL_MUJOCO_VISUALIZATION_UTILS_H

#include <mujoco.h>
#include <glfw3.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>

namespace mujoco_ros_control
{

class MujocoVisualizationUtils
{
public:
  static MujocoVisualizationUtils& getInstance()
  {
    static MujocoVisualizationUtils instance;
    return instance;
  }

  void init(mjModel* mujoco_model, mjData* mujoco_data, GLFWwindow* window);

  void update(GLFWwindow* window);
  
  void terminate();

  static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);

  void keyboard_cb_implementation(GLFWwindow* window, int key, int scancode, int act, int mods);

  static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);

  void mouse_move_cb_implementation(GLFWwindow* window, double xpos, double ypos);

  static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

  void scroll_cb_implementation(GLFWwindow* window, double xoffset, double yoffset);

  static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);

  void mouse_button_cb_implementation(GLFWwindow* window, int button, int act, int mods);

private:
  MujocoVisualizationUtils(void) // private constructor necessary to allow only 1 instance
  {};

  MujocoVisualizationUtils(MujocoVisualizationUtils const&); // prevent copies
  void operator=(MujocoVisualizationUtils const&); // prevent assignments

protected:

  // MuJoCo data structures
  mjModel* mujoco_model_;
  mjData* mujoco_data_;
  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context
  mjvPerturb pert;
  mjvFigure figconstraint;
  mjvFigure figcost;
  mjvFigure figtimer;
  mjvFigure figsize;
  mjvFigure figsensor;

  // mouse interaction
  bool button_left;
  bool button_middle;
  bool button_right ;
  double lastx;
  double lasty;
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_MUJOCO_VISUALIZATION_UTILS_H