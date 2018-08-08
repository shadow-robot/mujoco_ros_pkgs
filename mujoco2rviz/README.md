# mujoco2rviz

This package contains source code for translating models spawned in Mujoco simulator to rviz. To get the objects it uses *mujoco/model_states* topic published by [mujoco_ros_control](https://github.com/shadow-robot/mujoco_ros_pkgs/tree/kinetic-devel/mujoco_ros_control).

## Usage

In order to start the node, run:

```sh
roslaunch mujoco2rviz mujoco2rviz.launch
```
By default, only static objects are translated to rviz. In order to include free objects as well, use the command above with `static_only:=false` argument.

## Free/static object distinction

All objects that have no joints are considered static. Objects that have exactly one free joint are considered free.