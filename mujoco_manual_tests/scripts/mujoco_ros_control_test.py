#!/usr/bin/env python

# Use with: roslaunch mujoco_manual_tests multiple_type_objects.launch


import argparse
import rospy
from geometry_msgs.msg import Pose
from mujoco_ros_msgs.msg import ModelStates
from sr_utilities_common.manual_test_suite import ManualTestSuite


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


class MujocoRosControlTest(object):
    def __init__(self):
        self.published_objects = rospy.wait_for_message('/mujoco/model_states', ModelStates)
        self.ignore_unknown_type_objects()

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

    def ignore_unknown_type_objects(self):
        for idx, object_type in enumerate(self.published_objects.type):
            if 'unknown_type' == object_type:
                del self.published_objects.name[idx]
                del self.published_objects.type[idx]
                del self.published_objects.is_static[idx]
                del self.published_objects.size[idx]
                del self.published_objects.pose[idx]

    def test_objects_in_message(self):
        result = True
        for object_name in self.objects_test_case:
            if object_name not in self.published_objects.name:
                result = False
                break
        return result

    def test_object_types(self):
        result = True
        for idx, object_type in enumerate(self.published_objects.type):
            if object_type != self.objects_test_case[self.published_objects.name[idx]]['type']:
                result = False
                break
        return result

    def test_if_objects_static(self):
        result = True
        for idx, is_static in enumerate(self.published_objects.is_static):
            if is_static != self.objects_test_case[self.published_objects.name[idx]]['is_static']:
                result = False
                break
        return result

    def test_object_poses(self):
        result = True
        for idx, pose in enumerate(self.published_objects.pose):
            if not compare_poses(pose, list_to_pose(self.objects_test_case[self.published_objects.name[idx]]['pose'])):
                print "pose of {} different!".format(self.published_objects.name[idx])
                result = False
                break
        return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('mujoco_ros_control_test')
    mujoco_ros_control_test = MujocoRosControlTest()
    test_suite = ManualTestSuite(mujoco_ros_control_test, unattended=args.unattended)
    test_suite.create_test('test_objects_in_message()')
    test_suite.create_test('test_object_types()')
    test_suite.create_test('test_if_objects_static()')
    test_suite.create_test('test_object_poses()')
    test_suite.print_summary()
