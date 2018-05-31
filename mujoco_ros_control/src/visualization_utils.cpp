/*
 * Copyright (c) 2018, Shadow Robot Company, All rights reserved.
 *
 * @file   mujoco_ros_control.cpp
 * @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
 * @brief  Hardware interface for simulated robot in Mujoco
 **/

#include <mujoco_ros_control/visualization_utils.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>
#include <string>
#include <iostream>

namespace mujoco_ros_control
{

//MujocoVisualizationUtils::~MujocoVisualizationUtils()
//{};

void MujocoVisualizationUtils::init(mjModel* mujoco_model, mjData* mujoco_data, GLFWwindow* window)
{
  // save references
  mujoco_model_ = mujoco_model;
  mujoco_data_ = mujoco_data;

  // MuJoCo data structures
  mjvCamera cam;                      // abstract camera
  mjvOption opt;                      // visualization options
  mjvScene scn;                       // abstract scene
  mjrContext con;                     // custom GPU context

  // mouse interaction
  button_left = false;
  button_middle = false;
  button_right =  false;
  lastx = 0;
  lasty = 0;

  // initialize MuJoCo visualization
  mjv_makeScene(&scn, 1000);
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);
  mjr_makeContext(mujoco_model_, &con, 200);

  // center and scale view
  cam.lookat[0] = mujoco_model_->stat.center[0];
  cam.lookat[1] = mujoco_model_->stat.center[1];
  cam.lookat[2] = mujoco_model_->stat.center[2];
  cam.distance = 1.5 * mujoco_model_->stat.extent;

  //boost::function<void (GLFWwindow*, int, int, int, int)> keyboard_cb = boost::bind(&MujocoVisualizationUtils::keyboard, this, _1);
  glfwSetKeyCallback(window, &MujocoVisualizationUtils::keyboard_callback);
  //glfwSetCursorPosCallback(window, mouse_move);
  //glfwSetScrollCallback(window, scroll);
}

void MujocoVisualizationUtils::update(GLFWwindow* window)
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(mujoco_model_, mujoco_data_, &opt,
                  NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoVisualizationUtils::terminate()
{
  // free scene and terminate glfw
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  glfwTerminate();
}

void MujocoVisualizationUtils::keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  //here we access the instance via the singleton pattern and forward the callback to the instance method
  getInstance().keyboard_cb_implementation(window, key, scancode, act, mods);
}

// keyboard callback
void MujocoVisualizationUtils::keyboard_cb_implementation(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
  {
    mj_resetData(mujoco_model_, mujoco_data_);
    mj_forward(mujoco_model_, mujoco_data_);
  }
}

void MujocoVisualizationUtils::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
  //here we access the instance via the singleton pattern and forward the callback to the instance method
  getInstance().mouse_move_cb_implementation(window, xpos, ypos);
}

// mouse move callback
void MujocoVisualizationUtils::mouse_move_cb_implementation(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if( !button_left && !button_middle && !button_right )
      return;

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if( button_right )
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if( button_left )
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
      action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(mujoco_model_, action, dx/height, dy/height, &scn, &cam);
}

void MujocoVisualizationUtils::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
  //here we access the instance via the singleton pattern and forward the callback to the instance method
  getInstance().scroll_cb_implementation(window, xoffset, yoffset);
}

// scroll callback
void MujocoVisualizationUtils::scroll_cb_implementation(GLFWwindow* window, double xoffset, double yoffset)
{
  // require model
  if( !mujoco_model_ )
      return;
  // scroll: emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mujoco_model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

}  // namespace mujoco_ros_control
