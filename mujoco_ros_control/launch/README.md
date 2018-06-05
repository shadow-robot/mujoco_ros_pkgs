# UTL5 Mujoco Demo

## Overview and Setup

The goal of the demo is to evaluate Mujoco performance in performing utl5_demo in simulation.
In order to run the demo the following packages are needed:

- [sr_utl_demos](https://github.com/shadow-robot/sr_utl_demos)
- [shadow's fork of the object_recognition_msgs](https://github.com/shadow-robot/object_recognition_msgs)
- Switch sr_manipulation to F#SRC-1374_mujoco_test_debug branch (The only difference between this branch and kinetic-devel is to separate the approach and grasp phase which is useful for debugging reasons)

Before running the demo create the db folder in the tmp directory of your Docker with the following command:
```mkdir /tmp/db
```

## Usage

To launch the simulation and load the models and necessary grasps run the following launch file:

```
roslaunch mujoco_ros_control demo_grasp.launch sim:=true grasp_controller:=true
```

Since we are not using vision we need to mock the publishing of the TF we want to grasp. This is achieved by running the following script:

```
rosrun sr_vision_mocks spawn_object.py -p "utl5_large" 0 0.7 0 0.4 1.57 0
```

In which utl5_large is the name of the object to pick and the other six fields represent the pose of the object.
The pose corresponds with pose of the object defined in the [mujoco description file](https://github.com/shadow-robot/mujoco_ros_pkgs/blob/F%23SRC-1374_grasp_pipeline/mujoco_models/urdf/ur10_fh_environment.xml#L42). Although the z is set up higher since in Mujoco the frame of
the object is placed at the center of the object, while in the normal grasping pipeline this is placed on the surface.

To start the proper pipeline run the pick_place server with the following command:

```
rosrun sr_manipulation_grasp_conductor pick_place_server.py
```

When a message `Ready to receive place goals` appears, the pipeline is ready to pick the boxes. To initiate grasping, run the following:

```
rosrun mujoco_ros_control pick_and_place_client.py -o "utl5_large_0" -p 0.761 0.873 0.7633 0.0 0.0 1.5707963267
```

Here -o option represents the id of the object to pick up as defined in the recognized_object message and -p option represents the final place pose
