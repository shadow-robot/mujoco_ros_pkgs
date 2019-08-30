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
* @file   mujoco_visualization_utils.h
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
**/

#ifndef MUJOCO_ROS_CONTROL_VISUALIZATION_UTILS_H
#define MUJOCO_ROS_CONTROL_VISUALIZATION_UTILS_H

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

private:
  MujocoVisualizationUtils(void)
  {};

  MujocoVisualizationUtils(MujocoVisualizationUtils const&);
  void operator=(MujocoVisualizationUtils const&);

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

  char lastfile[1000];
  char opt_title[1000];
  char opt_content[1000];
  char status[1000];

  // user state
  bool paused;
  bool showoption;
  bool showinfo;
  bool showfullscreen;
  bool slowmotion;
  bool showdepth;
  bool showsensor;
  bool showprofiler;
  int showhelp;
  int fontscale;
  int keyreset;

  // mouse interaction
  bool button_left;
  bool button_middle;
  bool button_right;
  double lastx;
  double lasty;

  static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);

  void keyboard_cb_implementation(GLFWwindow* window, int key, int scancode, int act, int mods);

  static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);

  void mouse_move_cb_implementation(GLFWwindow* window, double xpos, double ypos);

  static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

  void scroll_cb_implementation(GLFWwindow* window, double xoffset, double yoffset);

  static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);

  void mouse_button_cb_implementation(GLFWwindow* window, int button, int act, int mods);

  void profiler_init();

  void profiler_update();

  void profiler_show(mjrRect rect);

  void sensor_init();

  void sensor_update();

  void sensor_show(mjrRect rect);

  mjtNum timer();

  void clear_timers(mjData* mujoco_data);

  void autoscale(GLFWwindow* window);
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_VISUALIZATION_UTILS_H
