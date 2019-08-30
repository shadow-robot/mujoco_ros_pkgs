#!/usr/bin/env python
#
# Copyright 2018 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

# Use with: roslaunch mujoco_manual_tests multiple_type_objects.launch

import os
import time
import argparse
import rospy
import subprocess
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from sr_utilities_common.manual_test_suite import ManualTestSuite


class Mujoco2RvizTest(object):
    def __init__(self):
        self.process = None
        self.devnull = open(os.devnull, 'wb')
        self.static_objects = ['box_0__link', 'cylinder_0__link']
        self.free_objects = ['box_1__link', 'sphere_0__link']
        rospy.wait_for_service('get_planning_scene')
        self.plannig_scene_service = rospy.ServiceProxy('get_planning_scene',
                                                        GetPlanningScene)

    def _close_process(self):
        self.process.send_signal(subprocess.signal.SIGINT)

    def _get_all_collision_objects(self):
        collision_object_names = []
        planning_scene_request = PlanningSceneComponents()
        planning_scene_request.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        planning_scene = self.plannig_scene_service(planning_scene_request)
        for collision_object in planning_scene.scene.world.collision_objects:
            collision_object_names.append(collision_object.id)
        return collision_object_names

    def _check_if_planning_scene_empty(self, timeout=10):
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            planning_scene_objects = self._get_all_collision_objects()
            if not planning_scene_objects:
                return True
        return False

    def _check_if_published_objects_match_planning_scene(self, objects_list, timeout=10):
        timeout_time = time.time() + timeout
        while time.time() < timeout_time:
            result = True
            planning_scene_objects = self._get_all_collision_objects()
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

    def test_spawn_static_objects(self, timeout=20):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch'],
                                        shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)
        return self._check_if_published_objects_match_planning_scene(self.static_objects,
                                                                     timeout)

    def test_spawn_all_objects(self, timeout=20):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch static_only:=false'],
                                        shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)
        return self._check_if_published_objects_match_planning_scene(self.static_objects + self.free_objects,
                                                                     timeout)

    def test_clean_up_static_objects(self, timeout=20):
        self._close_process()
        return self._check_if_planning_scene_empty(timeout)

    def test_clean_up_all_objects(self, timeout=20):
        self._close_process()
        return self._check_if_planning_scene_empty(timeout)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('mujoco2rvizl_test')
    mujoco2rviz_test = Mujoco2RvizTest()
    ordered_test_method_list = ['test_spawn_static_objects',
                                'test_clean_up_static_objects',
                                'test_spawn_all_objects',
                                'test_clean_up_all_objects']
    test_suite = ManualTestSuite(mujoco2rviz_test, ordered_test_method_list, unattended=args.unattended)
