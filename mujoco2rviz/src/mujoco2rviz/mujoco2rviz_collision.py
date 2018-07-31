#!/usr/bin/env python

import rospy
import rospkg
import re

import argparse
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from mujoco_ros_msgs.msg import FreeObjectsStates
from mujoco2rviz.utilities import compare_poses, stl_to_mesh, get_object_mesh_path, get_object_type_from_name

class Mujoco2Marker():
    def __init__(self):
        self.model_cache = {}
        self.description_repo_path = rospy.get_param("description_repo_path",
                                                     rospkg.RosPack().get_path('sr_utl5'))
        self.objects_states_subscriber = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)
        self.collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=5,
                                                          latch=True)
        self.publish_objects_to_rviz()

    def objects_states_cb(self, objects_states_msg):
        '''

        '''
        for model_instance_name, model_instance_pose in zip(objects_states_msg.name, objects_states_msg.pose):
            if not model_instance_name in self.model_cache:
                try:
                    self.model_cache[model_instance_name] = self.create_collision_object(model_instance_name, model_instance_pose)
                    rospy.loginfo("Added object {} to rviz".format(model_instance_name))
                except:
                    rospy.logwarn("Failed to add {} collision object".format(model_instance_name))

            if not compare_poses(model_instance_pose, self.model_cache[model_instance_name].mesh_poses[0]):
                object_to_be_temporarily_removed = self.create_collision_object(model_instance_name, model_instance_pose, False)
                self.collision_object_publisher.publish(object_to_be_temporarily_removed)
                self.model_cache[model_instance_name].mesh_poses[0] = model_instance_pose

    def publish_objects_to_rviz(self):
        while not rospy.is_shutdown():
            for model_instance_name in self.model_cache.keys():
                self.collision_object_publisher.publish(self.model_cache[model_instance_name])

    def create_collision_object(self, model_instance_name, model_pose, add=True):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        if add:
            object_type = get_object_type_from_name(model_instance_name)
            object_mesh_path = get_object_mesh_path(object_type, self.description_repo_path)
            collision_object.operation = CollisionObject.ADD
            object_mesh = stl_to_mesh(object_mesh_path)
            collision_object.meshes = [object_mesh]
            collision_object.mesh_poses = [model_pose]
        else:
            collision_object.operation = CollisionObject.REMOVE
        return collision_object
