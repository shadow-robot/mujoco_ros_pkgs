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

void MujocoVisualizationUtils::init(mjModel* mujoco_model, mjData* mujoco_data, GLFWwindow* window)
{
  ROS_INFO("Initializing GL functions");
  // save references
  mujoco_model_ = mujoco_model;
  mujoco_data_ = mujoco_data;

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

  glfwSetMouseButtonCallback(window,  &MujocoVisualizationUtils::mouse_button_callback);
  glfwSetKeyCallback(window, &MujocoVisualizationUtils::keyboard_callback);
  glfwSetCursorPosCallback(window, &MujocoVisualizationUtils::mouse_move_callback);
  glfwSetScrollCallback(window, &MujocoVisualizationUtils::scroll_callback);
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
  getInstance().keyboard_cb_implementation(window, key, scancode, act, mods);
}

// keyboard callback
void MujocoVisualizationUtils::keyboard_cb_implementation(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mujoco_model_, mujoco_data_);
    mj_forward(mujoco_model_, mujoco_data_);
  }
}

void MujocoVisualizationUtils::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
  getInstance().mouse_move_cb_implementation(window, xpos, ypos);
}

// mouse move callback
void MujocoVisualizationUtils::mouse_move_cb_implementation(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
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
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (button_left)
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
      action = mjMOUSE_ZOOM;

  // move camera
  mjv_moveCamera(mujoco_model_, action, dx/height, dy/height, &scn, &cam);
}

void MujocoVisualizationUtils::mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
  getInstance().mouse_button_cb_implementation(window, button, act, mods);
}

void MujocoVisualizationUtils::mouse_button_cb_implementation(GLFWwindow* window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // Alt: swap left and right
    if ((mods & GLFW_MOD_ALT))
    {
        bool tmp = button_left;
        button_left = button_right;
        button_right = tmp;

        if (button == GLFW_MOUSE_BUTTON_LEFT)
            button = GLFW_MOUSE_BUTTON_RIGHT;
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            button = GLFW_MOUSE_BUTTON_LEFT;
    }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if (!mujoco_model_)
        return;

    // set perturbation
    int newperturb = 0;
    if (act == GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select > 0)
    {
        // right: translate;  left: rotate
        if (button_right)
            newperturb = mjPERT_TRANSLATE;
        else if (button_left)
            newperturb = mjPERT_ROTATE;

        // perturbation onset: reset reference
        if (newperturb && !pert.active)
            mjv_initPerturb(mujoco_model_, mujoco_data_, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if (act == GLFW_PRESS && glfwGetTime()-lastclicktm < 0.25 && button == lastbutton)
    {
        // determine selection mode
        int selmode;
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            selmode = 1;
        else if (mods & GLFW_MOD_CONTROL)
            selmode = 3;
        else
            selmode = 2;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];
        int selgeom = mjv_select(mujoco_model_, mujoco_data_, &opt,
                                 (mjtNum)width/(mjtNum)height,
                                 (mjtNum)lastx/(mjtNum)width,
                                 (mjtNum)(height-lasty)/(mjtNum)height,
                                 &scn, selpnt);
        int selbody = (selgeom >=0 ? mujoco_model_->geom_bodyid[selgeom] : 0);

        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3)
        {
            // copy selpnt if geom clicked
            if (selgeom >= 0)
                mju_copy3(cam.lookat, selpnt);

            // switch to tracking camera
            if (selmode == 3 && selbody)
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if (selbody)
            {
                // record selection
                pert.select = selbody;
                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, mujoco_data_->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, mujoco_data_->xmat+9*pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if (act == GLFW_PRESS)
    {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}

void MujocoVisualizationUtils::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
  getInstance().scroll_cb_implementation(window, xoffset, yoffset);
}

// scroll callback
void MujocoVisualizationUtils::scroll_cb_implementation(GLFWwindow* window, double xoffset, double yoffset)
{
  // require model
  if (!mujoco_model_)
      return;
  // scroll: emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mujoco_model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

}  // namespace mujoco_ros_control
