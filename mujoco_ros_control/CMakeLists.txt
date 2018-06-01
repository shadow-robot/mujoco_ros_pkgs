cmake_minimum_required(VERSION 2.8.3)
project(mujoco_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  controller_manager
  control_toolbox
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf 
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} robot_hw_sim
)

## Specify additional locations of header files
include_directories( include
 ${Boost_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
 /home/user/mjpro150/include
)

add_library(robot_hw_sim src/robot_hw_sim.cpp)
target_link_libraries(robot_hw_sim ${catkin_LIBRARIES})

set(MUJOCO_LIB_PATH "/home/user/mjpro150/bin/")
find_library (MUJOCO NAMES mujoco150 PATHS ${MUJOCO_LIB_PATH} NO_DEFAULT_PATH)
find_library (GLFW NAMES libglfw.so.3 PATHS ${MUJOCO_LIB_PATH} NO_DEFAULT_PATH)
find_library (GLEW NAMES libglew.so PATHS ${MUJOCO_LIB_PATH} NO_DEFAULT_PATH)
## Declare a C++ library
add_executable(mujoco_ros_control src/mujoco_ros_control.cpp)
target_link_libraries(mujoco_ros_control ${catkin_LIBRARIES} ${GLFW} ${MUJOCO} robot_hw_sim ${GLEW} glut EGL X11 GL GLU)

## Install
#install(TARGETS ${PROJECT_NAME} robot_hw_sim
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(FILES robot_hw_sim_plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
