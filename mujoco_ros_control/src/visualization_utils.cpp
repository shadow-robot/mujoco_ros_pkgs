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

  // user state
  paused = false;
  showoption = false;
  showinfo = false;
  showfullscreen = false;
  slowmotion = false;
  showdepth = false;
  showsensor = false;
  showprofiler = false;
  showhelp = 2;                   // 0: none; 1: brief; 2: full
  fontscale = mjFONTSCALE_150;    // can be 100, 150, 200
  keyreset = -1;                  // non-negative: reset to keyframe

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
  profiler_init();
  sensor_init();
}

void MujocoVisualizationUtils::update(GLFWwindow* window)
{
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
  mjrRect smallviewport = viewport;

  // update scene and render
  mjv_updateScene(mujoco_model_, mujoco_data_, &opt,
                  NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // show profiler
  if (showprofiler)
  {
    if (!paused)
       profiler_update();
    profiler_show(viewport);
  }

  // show sensor
  if (showsensor)
  {
    if (!paused)
       sensor_update();
    sensor_show(smallviewport);
  }

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

void MujocoVisualizationUtils::keyboard_cb_implementation(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    int n;
    char error[1000];

    // require model
    if (!mujoco_model_)
        return;

    // do not act on release
    if (act == GLFW_RELEASE)
        return;

    switch (key)
    {
    case GLFW_KEY_F1:                   // help
        mj_saveLastXML("/home/user/compiled_model", mujoco_model_, error, 1000);
        showhelp++;
        if (showhelp > 2)
            showhelp = 0;
        break;

    case GLFW_KEY_F2:                   // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3:                   // info
        showinfo = !showinfo;
        break;

    case GLFW_KEY_F4:                   // depth
        showdepth = !showdepth;
        break;

    case GLFW_KEY_F6:                   // stereo
        scn.stereo = (scn.stereo == mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
        break;

    case GLFW_KEY_F7:                   // sensor figure
        showsensor = !showsensor;
        break;

    case GLFW_KEY_F8:                   // profiler
        showprofiler = !showprofiler;
        break;

    case GLFW_KEY_ENTER:                // slow motion
        slowmotion = !slowmotion;
        break;

    case GLFW_KEY_SPACE:                // pause
        paused = !paused;
        break;

    case GLFW_KEY_PAGE_UP:              // previous keyreset
    case GLFW_KEY_PAGE_DOWN:            // next keyreset
        if (key == GLFW_KEY_PAGE_UP)
            keyreset = mjMAX(-1, keyreset-1);
        else
            keyreset = mjMIN(mujoco_model_->nkey-1, keyreset+1);

    // continue with reset

    case GLFW_KEY_BACKSPACE:            // reset
        mj_resetData(mujoco_model_, mujoco_data_);
        if (keyreset >= 0 && keyreset < mujoco_model_->nkey)
        {
            mujoco_data_->time = mujoco_model_->key_time[keyreset];
            mju_copy(mujoco_data_->qpos, mujoco_model_->key_qpos + keyreset*mujoco_model_->nq, mujoco_model_->nq);
            mju_copy(mujoco_data_->qvel, mujoco_model_->key_qvel + keyreset*mujoco_model_->nv, mujoco_model_->nv);
            mju_copy(mujoco_data_->act, mujoco_model_->key_act + keyreset*mujoco_model_->na, mujoco_model_->na);
        }
        mj_forward(mujoco_model_, mujoco_data_);
        profiler_update();
        sensor_update();
        break;

    case GLFW_KEY_RIGHT:                // step forward
        if (paused)
        {
            mj_step(mujoco_model_, mujoco_data_);
            profiler_update();
            sensor_update();
        }
        break;

    case GLFW_KEY_LEFT:                 // step back
        if (paused)
        {
            mujoco_model_->opt.timestep = -mujoco_model_->opt.timestep;
            clear_timers(mujoco_data_);
            mj_step(mujoco_model_, mujoco_data_);
            mujoco_model_->opt.timestep = -mujoco_model_->opt.timestep;
            profiler_update();
            sensor_update();
        }
        break;

    case GLFW_KEY_DOWN:                 // step forward 100
        if (paused)
        {
            clear_timers(mujoco_data_);
            for (n = 0; n < 100; n++)
                mj_step(mujoco_model_, mujoco_data_);
            profiler_update();
            sensor_update();
        }
        break;

    case GLFW_KEY_UP:                   // step back 100
        if (paused)
        {
            mujoco_model_->opt.timestep = -mujoco_model_->opt.timestep;
            clear_timers(mujoco_data_);
            for (n = 0; n < 100; n++)
                mj_step(mujoco_model_, mujoco_data_);
            mujoco_model_->opt.timestep = -mujoco_model_->opt.timestep;
            profiler_update();
            sensor_update();
        }
        break;

    case GLFW_KEY_ESCAPE:               // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '=':                           // bigger font
        if (fontscale < 200)
        {
            fontscale += 50;
            mjr_makeContext(mujoco_model_, &con, fontscale);
        }
        break;

    case '-':                           // smaller font
        if (fontscale > 100)
        {
            fontscale -= 50;
            mjr_makeContext(mujoco_model_, &con, fontscale);
        }
        break;

    case '[':                           // previous fixed camera or free
        if (mujoco_model_->ncam && cam.type == mjCAMERA_FIXED)
        {
            if (cam.fixedcamid > 0)
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']':                           // next fixed camera
        if (mujoco_model_->ncam)
        {
            if (cam.type != mjCAMERA_FIXED)
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if (cam.fixedcamid < mujoco_model_->ncam-1)
                cam.fixedcamid++;
        }
        break;

    case ';':                           // cycle over frame rendering modes
        opt.frame = mjMAX(0, opt.frame-1);
        break;

    case '\'':                          // cycle over frame rendering modes
        opt.frame = mjMIN(mjNFRAME-1, opt.frame+1);
        break;

    case '.':                           // cycle over label rendering modes
        opt.label = mjMAX(0, opt.label-1);
        break;

    case '/':                           // cycle over label rendering modes
        opt.label = mjMIN(mjNLABEL-1, opt.label+1);
        break;

    default:                            // toggle flag
        // control keys
        if (mods & GLFW_MOD_CONTROL)
        {
            if (key == GLFW_KEY_A)
                autoscale(window);
            break;
        }

        // toggle visualization flag
        for (int i = 0; i < mjNVISFLAG; i++)
            if (key == mjVISSTRING[i][2][0])
                opt.flags[i] = !opt.flags[i];

        // toggle rendering flag
        for (int i = 0; i < mjNRNDFLAG; i++)
            if (key == mjRNDSTRING[i][2][0])
                scn.flags[i] = !scn.flags[i];

        // toggle geom/site group
        for (int i = 0; i < mjNGROUP; i++)
            if (key == i+'0')
            {
                if (mods & GLFW_MOD_SHIFT)
                    opt.sitegroup[i] = !opt.sitegroup[i];
                else
                    opt.geomgroup[i] = !opt.geomgroup[i];
            }
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

// init profiler
void MujocoVisualizationUtils::profiler_init()
{
    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    snprintf(figconstraint.title, sizeof(figconstraint.title), "Counts");
    snprintf(figcost.title, sizeof(figcost.title), "Convergence (log 10)");
    snprintf(figsize.title, sizeof(figsize.title), "Dimensions");
    snprintf(figtimer.title, sizeof(figtimer.title), "CPU time (msec)");

    // x-labels
    snprintf(figconstraint.xlabel, sizeof(figconstraint.xlabel), "Solver iteration");
    snprintf(figcost.xlabel, sizeof(figcost.xlabel), "Solver iteration");
    snprintf(figsize.xlabel, sizeof(figsize.xlabel), "Video frame");
    snprintf(figtimer.xlabel, sizeof(figtimer.xlabel), "Video frame");

    // y-tick nubmer formats
    snprintf(figconstraint.yformat, sizeof(figconstraint.yformat), ".0f");
    snprintf(figcost.yformat, sizeof(figcost.yformat), ".1f");
    snprintf(figsize.yformat, sizeof(figsize.yformat), ".0f");
    snprintf(figtimer.yformat, sizeof(figtimer.yformat), ".2f");

    // colors
    figconstraint.figurergba[0]  = 0.1f;
    figcost.figurergba[2] =  0.2f;
    figsize.figurergba[0] =  0.1f;
    figtimer.figurergba[2] =  0.2f;

    // legends
    snprintf(figconstraint.linename[0], sizeof(figconstraint.linename[0]), "total");
    snprintf(figconstraint.linename[1], sizeof(figconstraint.linename[1]), "active");
    snprintf(figconstraint.linename[2], sizeof(figconstraint.linename[2]), "changed");
    snprintf(figconstraint.linename[3], sizeof(figconstraint.linename[3]), "evals");
    snprintf(figconstraint.linename[4], sizeof(figconstraint.linename[4]), "updates");
    snprintf(figcost.linename[0], sizeof(figcost.linename[0]), "improvement");
    snprintf(figcost.linename[1], sizeof(figcost.linename[1]), "gradient");
    snprintf(figcost.linename[2], sizeof(figcost.linename[2]), "lineslope");
    snprintf(figsize.linename[0], sizeof(figsize.linename[0]), "dof");
    snprintf(figsize.linename[1], sizeof(figsize.linename[1]), "body");
    snprintf(figsize.linename[2], sizeof(figsize.linename[2]), "constraint");
    snprintf(figsize.linename[3], sizeof(figsize.linename[3]), "sqrt(nnz)");
    snprintf(figsize.linename[4], sizeof(figsize.linename[4]), "contact");
    snprintf(figsize.linename[5], sizeof(figsize.linename[5]), "iteration");
    snprintf(figtimer.linename[0], sizeof(figtimer.linename[0]), "total");
    snprintf(figtimer.linename[1], sizeof(figtimer.linename[1]), "collision");
    snprintf(figtimer.linename[2], sizeof(figtimer.linename[2]), "prepare");
    snprintf(figtimer.linename[3], sizeof(figtimer.linename[3]), "solve");
    snprintf(figtimer.linename[4], sizeof(figtimer.linename[4]), "other");

    // grid sizes
    figconstraint.gridsize[0] = 5;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = 0;
    figconstraint.range[0][1] = 20;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 80;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = 100;
    figtimer.range[0][0] = -200;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 0.4f;

    // init x axis on history figures (do not show yet)
    for (n = 0; n < 6; n++)
        for (i = 0; i < mjMAXLINEPNT; i++)
        {
            figtimer.linedata[n][2*i] = static_cast<float>(-i);
            figsize.linedata[n][2*i] = static_cast<float>(-i);
        }
}


// show profiler
void MujocoVisualizationUtils::profiler_update(void)
{
    int i, n;

    // update constraint figure
    figconstraint.linepnt[0] = mjMIN(mjMIN(mujoco_data_->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 5; i++)
        figconstraint.linepnt[i] = figconstraint.linepnt[0];
    if (mujoco_model_->opt.solver == mjSOL_PGS)
    {
        figconstraint.linepnt[3] = 0;
        figconstraint.linepnt[4] = 0;
    }
    if (mujoco_model_->opt.solver == mjSOL_CG)
        figconstraint.linepnt[4] = 0;
    for (i=0; i < figconstraint.linepnt[0]; i++)
    {
        // x
        figconstraint.linedata[0][2*i] = static_cast<float>(i);
        figconstraint.linedata[1][2*i] = static_cast<float>(i);
        figconstraint.linedata[2][2*i] = static_cast<float>(i);
        figconstraint.linedata[3][2*i] = static_cast<float>(i);
        figconstraint.linedata[4][2*i] = static_cast<float>(i);

        // y
        figconstraint.linedata[0][2*i+1] = static_cast<float>(mujoco_data_->nefc);
        figconstraint.linedata[1][2*i+1] = static_cast<float>(mujoco_data_->solver[i].nactive);
        figconstraint.linedata[2][2*i+1] = static_cast<float>(mujoco_data_->solver[i].nchange);
        figconstraint.linedata[3][2*i+1] = static_cast<float>(mujoco_data_->solver[i].neval);
        figconstraint.linedata[4][2*i+1] = static_cast<float>(mujoco_data_->solver[i].nupdate);
    }

    // update cost figure
    figcost.linepnt[0] = mjMIN(mjMIN(mujoco_data_->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 3; i++)
        figcost.linepnt[i] = figcost.linepnt[0];
    if (mujoco_model_->opt.solver == mjSOL_PGS)
    {
        figcost.linepnt[1] = 0;
        figcost.linepnt[2] = 0;
    }

    for (i = 0; i < figcost.linepnt[0]; i++)
    {
        // x
        figcost.linedata[0][2*i] = static_cast<float>(i);
        figcost.linedata[1][2*i] = static_cast<float>(i);
        figcost.linedata[2][2*i] = static_cast<float>(i);

        // y
        figcost.linedata[0][2*i+1] = static_cast<float>(mju_log10(mju_max(mjMINVAL,
                                                        mujoco_data_->solver[i].improvement)));
        figcost.linedata[1][2*i+1] = static_cast<float>(mju_log10(mju_max(mjMINVAL,
                                                        mujoco_data_->solver[i].gradient)));
        figcost.linedata[2][2*i+1] = static_cast<float>(mju_log10(mju_max(mjMINVAL,
                                                        mujoco_data_->solver[i].lineslope)));
    }

    // get timers: total, collision, prepare, solve, other
    int itotal = (mujoco_data_->timer[mjTIMER_STEP].duration > mujoco_data_->timer[mjTIMER_FORWARD].duration ?
                    mjTIMER_STEP : mjTIMER_FORWARD);
    float tdata[5] = {
        static_cast<float>((mujoco_data_->timer[itotal].duration/mjMAX(1, mujoco_data_->timer[itotal].number))),
        static_cast<float>((mujoco_data_->timer[mjTIMER_POS_COLLISION].duration/mjMAX(1,
                           mujoco_data_->timer[mjTIMER_POS_COLLISION].number))),
        static_cast<float>((mujoco_data_->timer[mjTIMER_POS_MAKE].duration/mjMAX(1,
                           mujoco_data_->timer[mjTIMER_POS_MAKE].number))) +
            static_cast<float>((mujoco_data_->timer[mjTIMER_POS_PROJECT].duration/mjMAX(1,
                               mujoco_data_->timer[mjTIMER_POS_PROJECT].number))),
        static_cast<float>((mujoco_data_->timer[mjTIMER_CONSTRAINT].duration/mjMAX(1,
                           mujoco_data_->timer[mjTIMER_CONSTRAINT].number))), 0
    };
    tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

    // update figtimer
    int pnt = mjMIN(201, figtimer.linepnt[0]+1);
    for (n = 0; n < 5; n++)
    {
        // shift data
        for (i = pnt-1; i > 0; i--)
            figtimer.linedata[n][2*i+1] = figtimer.linedata[n][2*i-1];

        // assign new
        figtimer.linepnt[n] = pnt;
        figtimer.linedata[n][1] = tdata[n];
    }

    // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
    float sdata[6] = {
        static_cast<float>(mujoco_model_->nv),
        static_cast<float>(mujoco_model_->nbody),
        static_cast<float>(mujoco_data_->nefc),
        static_cast<float>(mju_sqrt((mjtNum)mujoco_data_->solver_nnz)),
        static_cast<float>(mujoco_data_->ncon),
        static_cast<float>(mujoco_data_->solver_iter)
    };

    // update figsize
    pnt = mjMIN(201, figsize.linepnt[0]+1);
    for (n = 0; n < 6; n++)
    {
        // shift data
        for (i = pnt-1; i > 0; i--)
            figsize.linedata[n][2*i + 1] = figsize.linedata[n][2*i - 1];

        // assign new
        figsize.linepnt[n] = pnt;
        figsize.linedata[n][1] = sdata[n];
    }
}

// show profiler
void MujocoVisualizationUtils::profiler_show(mjrRect rect)
{
    mjrRect viewport = {rect.width - rect.width/5, rect.bottom, rect.width/5, rect.height/4};
    mjr_figure(viewport, &figtimer, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figsize, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figcost, &con);
    viewport.bottom += rect.height/4;
    mjr_figure(viewport, &figconstraint, &con);
}


// init sensor figure
void MujocoVisualizationUtils::sensor_init(void)
{
    // set figure to default
    mjv_defaultFigure(&figsensor);

    // set flags
    figsensor.flg_extend = 1;
    figsensor.flg_barplot = 1;

    // title
    snprintf(figsensor.title, sizeof(figsensor.title), "Sensor data");

    // y-tick nubmer format
    snprintf(figsensor.yformat, sizeof(figsensor.yformat), ".0f");

    // grid size
    figsensor.gridsize[0] = 2;
    figsensor.gridsize[1] = 3;

    // minimum range
    figsensor.range[0][0] = 0;
    figsensor.range[0][1] = 0;
    figsensor.range[1][0] = -1;
    figsensor.range[1][1] = 1;
}

// update sensor figure
void MujocoVisualizationUtils::sensor_update(void)
{
    static const int maxline = 10;

    // clear linepnt
    for (int i = 0; i < maxline; i++)
        figsensor.linepnt[i] = 0;

    // start with line 0
    int lineid = 0;

    // loop over sensors
    for (int n = 0; n < mujoco_model_->nsensor; n++)
    {
        // go to next line if type is different
        if (n > 0 && mujoco_model_->sensor_type[n] != mujoco_model_->sensor_type[n-1])
            lineid = mjMIN(lineid+1, maxline-1);

        // get info about this sensor
        mjtNum cutoff = (mujoco_model_->sensor_cutoff[n] > 0 ? mujoco_model_->sensor_cutoff[n] : 1);
        int adr = mujoco_model_->sensor_adr[n];
        int dim = mujoco_model_->sensor_dim[n];

        // data pointer in line
        int p = figsensor.linepnt[lineid];

        // fill in data for this sensor
        for (int i = 0; i < dim; i++)
        {
            // check size
            if ((p+2*i) >= mjMAXLINEPNT/2)
                break;

            // x
            figsensor.linedata[lineid][2*p+4*i] = static_cast<float>((adr+i));
            figsensor.linedata[lineid][2*p+4*i+2] = static_cast<float>((adr+i));

            // y
            figsensor.linedata[lineid][2*p+4*i+1] = 0;
            figsensor.linedata[lineid][2*p+4*i+3] = static_cast<float>((mujoco_data_->sensordata[adr+i]/cutoff));
        }

        // update linepnt
        figsensor.linepnt[lineid] = mjMIN(mjMAXLINEPNT-1,
                                          figsensor.linepnt[lineid]+2*dim);
    }
}

// show sensor figure
void MujocoVisualizationUtils::sensor_show(mjrRect rect)
{
    // render figure on the right
    mjrRect viewport = {rect.width - rect.width/4, rect.bottom, rect.width/4, rect.height/3};
    mjr_figure(viewport, &figsensor, &con);
}

// center and scale view
void MujocoVisualizationUtils::autoscale(GLFWwindow* window)
{
    // autoscale
    cam.lookat[0] = mujoco_model_->stat.center[0];
    cam.lookat[1] = mujoco_model_->stat.center[1];
    cam.lookat[2] = mujoco_model_->stat.center[2];
    cam.distance = 1.5 * mujoco_model_->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}

// timer in milliseconds
mjtNum MujocoVisualizationUtils::timer(void)
{
    // save start time
    static double starttm = 0;
    if (starttm == 0)
        starttm = glfwGetTime();

    // return time since start
    return (mjtNum)(1000 * (glfwGetTime() - starttm));
}


// clear all times
void MujocoVisualizationUtils::clear_timers(mjData* mujoco_data)
{
    for (int i = 0; i < mjNTIMER; i++)
    {
        mujoco_data->timer[i].duration = 0;
        mujoco_data->timer[i].number = 0;
    }
}

}  // namespace mujoco_ros_control
