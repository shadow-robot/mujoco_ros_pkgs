#!/usr/bin/env python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from unittest import TestCase
from geometry_msgs.msg import Pose
from mujoco_ros_msgs.msg import ModelStates

PKG = "mujoco_ros_control"

def list_to_pose(list):
    pose = Pose()
    pose.position.x = list[0]
    pose.position.y = list[1]
    pose.position.z = list[2]
    pose.orientation.x = list[3]
    pose.orientation.y = list[4]
    pose.orientation.z = list[5]
    pose.orientation.w = list[6]
    return pose


def compare_poses(pose_1, pose_2, epsilon_translation=0.1, epsilon_rotation=0.1):
    poses_equal = True
    if abs(pose_1.position.x - pose_2.position.x) > epsilon_translation or \
       abs(pose_1.position.y - pose_2.position.y) > epsilon_translation or \
       abs(pose_1.position.z - pose_2.position.z) > epsilon_translation or \
       abs(pose_1.orientation.x - pose_2.orientation.x) > epsilon_rotation or \
       abs(pose_1.orientation.y - pose_2.orientation.y) > epsilon_rotation or \
       abs(pose_1.orientation.z - pose_2.orientation.z) > epsilon_rotation or \
       abs(pose_1.orientation.w - pose_2.orientation.w) > epsilon_rotation:
        poses_equal = False
    return poses_equal


class TestMujocoRosControl(TestCase):
    def setUp(self):
        self.published_objects = rospy.wait_for_message('/mujoco/model_states', ModelStates)
        self._ignore_unknown_type_objects()
        self.objects_test_case = {'box_0': {'type': ModelStates.BOX,
                                            'is_static': True,
                                            'pose': [0, 0.7, 0.075, 0, 0, 0, 1]},
                                  'cylinder_0': {'type': ModelStates.CYLINDER,
                                                 'is_static': True,
                                                 'pose': [0.2, 0.7, 0.045, 0, 0, 0, 1]},
                                  'sphere_0': {'type': ModelStates.SPHERE,
                                                 'is_static': False,
                                                 'pose': [0.4, 0.7, 0.05, 0, 0, 0, 1]},
                                  'box_1': {'type': ModelStates.MESH,
                                            'is_static': False,
                                            'pose': [0.6, 0.7, 0, 0, 0, 0.707, 0.707]}}

    def _ignore_unknown_type_objects(self):
        for idx, object_type in enumerate(self.published_objects.type):
            if 'unknown_type' == object_type:
                del self.published_objects.name[idx]
                del self.published_objects.type[idx]
                del self.published_objects.is_static[idx]
                del self.published_objects.size[idx]
                del self.published_objects.pose[idx]

    def test_is_publishing(self):
        is_publishing = False
        try:
            msg = rospy.wait_for_message('/mujoco/model_states', ModelStates)
        except Exception:
            pass
        else:
            is_publishing = True  
        self.assertTrue(is_publishing)

    def test_objects_in_message(self):
        for object_name in self.objects_test_case:
            self.assertIn(object_name, self.published_objects.name)

    def test_object_types(self):
        for idx, object_type in enumerate(self.published_objects.type):
            self.assertEqual(object_type, self.objects_test_case[self.published_objects.name[idx]]['type'])

    def test_if_objects_static(self):
        for idx, is_static in enumerate(self.published_objects.is_static):
            self.assertEqual(is_static, self.objects_test_case[self.published_objects.name[idx]]['is_static'])

    def test_object_poses(self):
        for idx, pose in enumerate(self.published_objects.pose):
            self.assertTrue(compare_poses(pose, list_to_pose(self.objects_test_case[self.published_objects.name[idx]]['pose'])))

if __name__ == "__main__":
    import rostest

    rospy.init_node("test_mujoco_ros_control")
    rostest.rosrun(PKG, "test_mujoco_ros_control", TestMujocoRosControl)
