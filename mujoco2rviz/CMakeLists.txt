cmake_minimum_required(VERSION 2.8.3)
project(mujoco2rviz)
find_package(catkin REQUIRED COMPONENTS
  rospy
  mujoco_ros_msgs
)
catkin_python_setup()
