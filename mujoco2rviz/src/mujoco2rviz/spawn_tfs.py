#!/usr/bin/env python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
import tf2_ros
from mujoco_ros_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped

class Mujoco2RvizTfSpawner():
    def __init__(self):
        self.objects_to_poses_dict = {}
        self.supported_types = ['mesh']
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/mujoco/model_states", ModelStates, self.model_states_cb)

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

    def process_data(self, data):
        object_types = data.type
        for idx, object_type in enumerate(object_types):
            if object_type in self.supported_types:
                self.objects_to_poses_dict[data.name[idx]] = data.pose[idx]

if __name__ == '__main__':
    rospy.init_node('mujoco_spawn_tfs')
    mujoco2rviz_tf_spawner = Mujoco2RvizTfSpawner()
    rospy.spin()
