# UTL5 Mujoco Demo

## Overview and Setup

The goal of the demo is to evaluate Mujoco performance in performing utl5_demo in simulation.
In order to run the demo the following packages are needed:

- Use custom version of [fh_interface](https://github.com/shadow-robot/fh_interface/tree/F%23SRC-1374_mujoco_launch)
- [sr_utl_demos](https://github.com/shadow-robot/sr_utl_demos)
- [sr_vision](https://github.com/shadow-robot/sr_vision)
- [shadow's fork of the object_recognition_msgs](https://github.com/shadow-robot/object_recognition_msgs)
- Switch sr_manipulation to F#SRC-1374_mujoco_test_debug branch (The only difference between this branch and kinetic-devel is to separate the approach and grasp phase which is useful for debugging reasons)

Before running the demo create the db folder in the tmp directory of your Docker with the following command:
```
mkdir /tmp/db
```

## Usage

# Static objects demo

There are two possible static objects demos, one with the box and one with a cylinder.

To launch the box simulation demo and load the models and necessary grasps run the following launch file:

```
roslaunch mujoco_ros_utils demo_grasp.launch sim:=true grasp_controller:=true box:=true
```

To launch the cylinder simulation demo and load the models and necessary grasps run the following command:
```
roslaunch mujoco_ros_utils demo_grasp.launch sim:=true grasp_controller:=true cylinder:=true
```

Since we are not using vision we need to mock the publishing of the TF we want to grasp. 
This is achieved by running the following script for the box:

```
rosrun sr_vision_mocks spawn_object.py -p "utl5_large" 0 0.7 0 0.4 1.57 0
```

And the following one for the cylinder:
```
rosrun sr_vision_mocks spawn_object.py -p "utl5_large" 0 1 0 0.5 1.57 0
```

In which utl5_large is the name of the object to pick and the other six fields represent the pose of the object.
The pose corresponds with pose of the object defined in the [mujoco description file](https://github.com/shadow-robot/mujoco_ros_pkgs/blob/F%23SRC-1374_grasp_pipeline/mujoco_models/urdf/ur10_fh_environment.xml#L42). Although the z is set up higher since in Mujoco the frame of
the object is placed at the center of the object, while in the normal grasping pipeline this is placed on the surface.

To start the proper pipeline run the pick_place demo with the following command:

```
rosrun sr_utl5 pick_place_mujoco_demo.py
```

# Spawn object and place in tote demo

This demo involves picking and placing in a tote of a utl5_medium box.

First launch the following launch file by running this command on the terminal, this will start all the grasp related stuff and the mujoco service to spawn objects:

```
roslaunch mujoco_ros_utils tote_demo.launch
```

At this point call the spawn demo client by running the following command:

```
rosrun mujoco_ros_utils spawn_tote_demo.py
```

Once the simulation is up and running run the following command in your terminal:

```
rosrun sr_utl5 pick_place_client.py -o "utl5_medium_0" -p -0.7 0.7 0.1 0 0 0 -t 0.9
```