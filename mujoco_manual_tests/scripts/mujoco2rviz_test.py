#!/usr/bin/env python

# Use with: roslaunch mujoco_manual_tests multiple_type_objects.launch

import os
import time
import argparse
import rospy
import subprocess
from moveit_msgs.msg import CollisionObject
from moveit_python import PlanningSceneInterface
from sr_utilities_common.manual_test_suite import ManualTestSuite


class Mujoco2RvizTest(object):
    def __init__(self):
        self.process = None
        self.planning_scene_interface = PlanningSceneInterface('world')
        self.devnull = open(os.devnull, 'wb')
        self.static_objects = ['box_0__link', 'cylinder_0__link']
        self.free_objects = ['box_1__link', 'sphere_0__link']

    def _close_process(self):
        self.process.send_signal(subprocess.signal.SIGINT)

    def _check_if_planning_scene_empty(self, timeout=10):
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            if not self.planning_scene_interface.getKnownCollisionObjects():
                return True
        return False

    def _check_if_published_objects_match_planning_scene(self, objects_list, timeout=10):
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            result = True
            planning_scene_objects = self.planning_scene_interface.getKnownCollisionObjects()
            for collision_object in objects_list:
                if collision_object not in planning_scene_objects:
                    result = False
                    break
            if result:
                for collision_object in planning_scene_objects:
                    if collision_object not in objects_list:
                        result = False
                        break
            if result:
                break
        return result

    def test_spawn_static_objects(self, timeout=10):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch'],
                                         shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)
        return self._check_if_published_objects_match_planning_scene(self.static_objects,
                                                                     timeout)

    def test_spawn_all_objects(self, timeout=10):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch static_only:=false'],
                                         shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)
        return self._check_if_published_objects_match_planning_scene(self.static_objects + self.free_objects,
                                                                     timeout)

    def test_clean_up_static_objects(self, timeout=10):
        self._close_process()
        return self._check_if_planning_scene_empty(timeout)

    def test_clean_up_all_objects(self, timeout=10):
        self._close_process()
        return self._check_if_planning_scene_empty(timeout)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('mujoco2rvizl_test')
    mujoco2rviz_test = Mujoco2RvizTest()
    test_suite = ManualTestSuite(mujoco2rviz_test, unattended=args.unattended)
    test_suite.create_test('test_spawn_static_objects()')
    test_suite.create_test('test_clean_up_static_objects()', True)
    test_suite.create_test('test_spawn_all_objects()', True)
    test_suite.create_test('test_clean_up_all_objects()', True)
    test_suite.print_summary()
