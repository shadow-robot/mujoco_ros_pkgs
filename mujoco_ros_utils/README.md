# Mujoco ROS utils

Package containing utilities to use mujoco with ROS

## Spawn simulation with objects

This utils allows the user to automatically spawn instances of the Mujoco simulation with different objects.

### Launch
To start the utils run the following command:

```
roslaunch mujoco_ros_utils simulation_spawner.launch
```

This launch file can be called with the following arguments:

+ **mesh_directory**: absolute path to the directory containing the meshes of objects that user wants to spawn(default is mujoco_models folder)
+ **generated_mujoco_env_filename**: name of the generated mujoco file (default is called test.xml)
+ **base_mujoco_env_filename**: name of the base muojoco file (default loads ur10 + hand H)


This launch file launches the [spawn_simulation](https://github.com/shadow-robot/mujoco_ros_pkgs/blob/F%23SRC-1866_object_spawn_service/mujoco_ros_utils/scripts/spawn_simulation.py) node 
which runs two services:

+ **/mujoco/spawn_sim_environment**: this service requests an array of objects (msg type [RecognizedObjectsArray](http://docs.ros.org/kinetic/api/object_recognition_msgs/html/msg/RecognizedObjectArray.html)).
For each object both name (must correspond to the name of the stl file of the object) and pose must be defined.<br/>
Once the request is received, a script modifies the **base_mujoco_env_filename**, appending the xml tags necessary to spawn the wanted objects in Mujoco.</br>
Next, the mujoco simulation is launched by running [fh_ur10_and_hand2_mujoco.launch](https://github.com/shadow-robot/fh_interface/blob/F%23SRC-1374_mujoco_launch/fh_robot_launch/launch/fh_ur10_and_fh2_mujoco.launch) in a new instance of xterm.
The **generated_mujoco_env_filename** is passed as an argument to the launch file to load the new Mujoco environment.</br>
The service responds with a boolean returning whether the simulation has been correctly spawned or not.

+ **/mujoco/terminate_sim**: when this service is called, the Mujoco simulation previously spawned gets terminated.<br/>
It responds with a boolean returning whether the simulation has been correctly terminated or not.

### Client example

An example on how to interact with the spawn_simulation node can be found [here](https://github.com/shadow-robot/mujoco_ros_pkgs/blob/F%23SRC-1866_object_spawn_service/mujoco_ros_utils/scripts/spawn_sim_client_example.py).</br>
This can be tested by running the following command:
```
./spawn_sim_client_example.py
```

In this script 2 objects (finger_base_links) are defined with name and pose and pushed to an array of type RecognizedObjectsArray.</br> 
The request is then sent to the **/mujoco/spawn_sim_environment** service and the simulation is spawned with the objects in 
the requested pose as shown in the picture below:

![alt text](https://github.com/shadow-robot/mujoco_ros_pkgs/blob/F%23SRC-1866_object_spawn_service/mujoco_ros_utils/spaw_objects_example.png)





