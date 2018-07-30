#!/usr/bin/env python

import rospy
import os
import tf
import argparse
import pysdf
from mujoco_ros_msgs.msg import FreeObjectsStates
from visualization_msgs.msg import Marker
from sdf2rviz_collision import Sdf2moveit

class Mujoco2Marker():
    def __init__(self):
        self.sdf2moveit = Sdf2moveit()
        self.model_cache = {}
        self.objectsStatesSub = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)

    def objects_states_cb(self, objects_states_msg):
        for model_idx, model_instance_name in enumerate(objects_states_msg.name):
            model_name = pysdf.name2modelname(model_instance_name)

            if not model_instance_name in self.model_cache:
                self.model_cache[model_instance_name] = self.sdf2moveit.add_new_collision_object(model_name, model_instance_name)

            self.sdf2moveit.update_collision_object_with_pose(self.model_cache[model_instance_name], model_instance_name, objects_states_msg.pose[model_idx])

        for model_instance_name in list(self.model_cache):
            if model_instance_name not in objects_states_msg.name:
                rospy.loginfo("Object %s deleted from gazebo" % model_instance_name)
                self.sdf2moveit.delete_collision_object(model_instance_name)
                del self.model_cache[model_instance_name]