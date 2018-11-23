#!/usr/bin/env python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import tf2_ros
from math import sin, cos
from mujoco_ros_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion


class Mujoco2RvizTfSpawner():
    def __init__(self):
        self.objects_to_poses_dict = {}
        self.supported_types = ['mesh', 'box']
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/mujoco/model_states", ModelStates, self.model_states_cb, queue_size=1)

    def model_states_cb(self, data):
        self.process_data(data)
        for key, value in self.objects_to_poses_dict.iteritems():
            tf_to_be_published = self.object_pose_quaternion_to_tf(key, value)
            self.tf_broadcaster.sendTransform(tf_to_be_published)

    def object_pose_quaternion_to_tf(self, object_name, object_pose, parent_frame='world'):
        tf = TransformStamped()

        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = object_name
        tf.transform.translation.x = object_pose.position.x
        tf.transform.translation.y = object_pose.position.y
        tf.transform.translation.z = object_pose.position.z
        tf.transform.rotation.x = object_pose.orientation.x
        tf.transform.rotation.y = object_pose.orientation.y
        tf.transform.rotation.z = object_pose.orientation.z
        tf.transform.rotation.w = object_pose.orientation.w

        return tf

    def move_pose_alongside_intrinsic_z_axis(self, pose, distance):
        orientation_euler = euler_from_quaternion([pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w])
        translation_normal_vector = [sin(orientation_euler[1]),
                                     -sin(orientation_euler[0])*cos(orientation_euler[1]),
                                     cos(orientation_euler[0])*cos(orientation_euler[1])]

        translation_vector = [coord * distance for coord in translation_normal_vector]
        pose.position.x += translation_vector[0]
        pose.position.y += translation_vector[1]
        pose.position.z += translation_vector[2]

        return pose

    def process_data(self, data):
        object_types = data.type
        for idx, object_type in enumerate(object_types):
            if object_type in self.supported_types:
                self.objects_to_poses_dict[data.name[idx]] = data.pose[idx]
                if 'box' == object_type:
                    # Move frame to the bottom of the object
                    object_name = data.name[idx]
                    object_height = data.size[idx].data[2] * 2
                    self.objects_to_poses_dict[data.name[idx]] = self.move_pose_alongside_intrinsic_z_axis(
                                                                    self.objects_to_poses_dict[object_name],
                                                                                               -object_height/2)

if __name__ == '__main__':
    rospy.init_node('mujoco_spawn_tfs')
    mujoco2rviz_tf_spawner = Mujoco2RvizTfSpawner()
    rospy.spin()
