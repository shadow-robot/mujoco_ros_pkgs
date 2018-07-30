#!/usr/bin/env python

import rospy
import os
import tf
import argparse
import pysdf
from mujoco_ros_msgs.msg import FreeObjectsStates
from visualization_msgs.msg import Marker
from mujoco2rviz.conversions import link2marker_msg
from sdf2moveit_collision_node import Sdf2moveit

class Mujoco2Marker():
    def __init__(self):
        self.objectsStatesSub = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)
        self.markerPub = rospy.Publisher('/visualization_marker', Marker)
        self.model_cache = {}
        self.update_period = 0.5
        self.use_collision = True # parse this

    def publish_link_marker(self, link, full_link_name, **kwargs):
        if 'model_name' in kwargs and 'instance_name' in kwargs:
            full_link_name = full_link_name.replace(kwargs['model_name'], kwargs['instance_name'], 1)
        marker_msgs = link2marker_msg(link, full_link_name, self.use_collision, rospy.Duration(2 * self.update_period))
        if len(marker_msgs) > 0:
            for marker_msg in marker_msgs:
                self.markerPub.publish(marker_msg)

    def objects_states_cb(self, objects_states_msg):
        for model_idx, model_instance_name in enumerate(objects_states_msg.name):
            model_name = pysdf.name2modelname(model_instance_name)
            if not model_name in self.model_cache:
                sdf = pysdf.SDF(model=model_name)
                if len(sdf.world.models) >= 1:
                    rospy.loginfo('Loaded model: {}'.format(model_name))
                    self.model_cache[model_name] = sdf.world.models[0]
                    self.model_cache[model_name].for_all_links(self.publish_link_marker, model_name=model_name, instance_name=model_instance_name)
                else:
                    rospy.logwarn('Unable to load model: %s' % model_name)