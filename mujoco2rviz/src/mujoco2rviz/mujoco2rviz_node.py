#!/usr/bin/env python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import rospkg
import re
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from mujoco_ros_msgs.msg import FreeObjectsStates
from shape_msgs.msg import SolidPrimitive
from mujoco2rviz.utilities import compare_poses, stl_to_mesh, get_object_mesh_path, get_object_name_from_instance


class Mujoco2Rviz():
    def __init__(self):
        self.model_cache = {}
        self.description_repo_path = rospy.get_param('~description_repo_path',
                                                     rospkg.RosPack().get_path('sr_description_common'))
        self.objects_states_subscriber = rospy.Subscriber('mujoco/free_objects_states', FreeObjectsStates, self.objects_states_cb)
        self.collision_object_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=5,
                                                          latch=True)
        self.publish_objects_to_rviz()

    def objects_states_cb(self, objects_states_msg):
        for model_idx, model_instance_name in enumerate(objects_states_msg.name):
            # create collision object if name is not present in model_cache
            if not model_instance_name in self.model_cache:
                try:
                    self.model_cache[model_instance_name] = self.create_collision_object_from_mujoco_msg(objects_states_msg, model_idx)
                    rospy.loginfo("Added object {} to rviz".format(model_instance_name))
                except:
                    rospy.logwarn("Failed to add {} collision object".format(model_instance_name))

            # check if model moved, if true update pose in model_cache
            if 'mesh' == objects_states_msg.type[model_idx]:
                if not compare_poses(objects_states_msg.pose[model_idx], self.model_cache[model_instance_name].mesh_poses[0]):
                    self.model_cache[model_instance_name].operation = CollisionObject.MOVE
                    self.model_cache[model_instance_name].mesh_poses[0] = objects_states_msg.pose[model_idx]
            else:
                if not compare_poses(objects_states_msg.pose[model_idx], self.model_cache[model_instance_name].primitive_poses[0]):
                    self.model_cache[model_instance_name].operation = CollisionObject.MOVE
                    self.model_cache[model_instance_name].primitive_poses[0] = objects_states_msg.pose[model_idx]

    def publish_objects_to_rviz(self):
        while not rospy.is_shutdown():
            for model_instance_name in self.model_cache.keys():
                self.collision_object_publisher.publish(self.model_cache[model_instance_name])

    def create_collision_object_from_mujoco_msg(self, message, model_idx):
        if 'mesh' == message.type[model_idx]:
            collision_object = self.create_collision_object_from_mesh(message.name[model_idx],
                                                                      message.pose[model_idx])
        else:
            collision_object = self.create_collision_object_from_primitive(message.name[model_idx],
                                                                           message.pose[model_idx],
                                                                           message.type[model_idx],
                                                                           message.size[model_idx].data)
        return collision_object

    def create_collision_object_from_mesh(self, model_instance_name, model_pose):
        collision_object = self.create_collision_object_base(model_instance_name)
        object_type = get_object_name_from_instance(model_instance_name)
        object_mesh_path = get_object_mesh_path(object_type, self.description_repo_path)
        try:
            object_mesh = stl_to_mesh(object_mesh_path)
        except:
            rospy.logwarn("Failed to transform mesh")
        collision_object.meshes = [object_mesh]
        collision_object.mesh_poses = [model_pose]
        return collision_object

    def create_collision_object_from_primitive(self, model_instance_name, model_pose, model_type, size):
        collision_object = self.create_collision_object_base(model_instance_name)
        primitive = SolidPrimitive()
        if 'box' == model_type:
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [i * 2 for i in size]
        elif 'cylinder' == model_type:
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [size[1] * 2, size[0]]
        elif 'sphere' == model_type:
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [size[0]]
        else:
            rospy.logerr("Primitive type {} not supported".format(model_type))
            return None
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(model_pose)
        return collision_object

    def create_collision_object_base(self, model_instance_name):
        collision_object = CollisionObject()
        collision_object.header.frame_id = 'world'
        collision_object.id = '{}__link'.format(model_instance_name)
        collision_object.operation = CollisionObject.ADD
        return collision_object


if __name__ == '__main__':
    rospy.init_node('mujoco_to_rviz', anonymous=True)
    m2m = Mujoco2Rviz()
    