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

namespace mujoco_ros_control
{

class MujocoVisualizationUtils
{
public:
  virtual ~MujocoVisualizationUtils();

  void init(mjModel* mujoco_model, mjData* mujoco_data, GLFWwindow* window);

  void update(GLFWwindow* window);
  
  void terminate();

  void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);

  void mouse_move(GLFWwindow* window, double xpos, double ypos);

  void scroll(GLFWwindow* window, double xoffset, double yoffset);
  
protected:
  
  // MuJoCo data structures
  mjModel* mujoco_model_;
  mjData* mujoco_data_;
  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context

  // mouse interaction
  bool button_left;
  bool button_middle;
  bool button_right ;
  double lastx;
  double lasty;
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_MUJOCO_VISUALIZATION_UTILS_H