#!/usr/bin/env python

import os
import argparse
import rospy
import subprocess
from sr_utilities_common.manual_test_suite import ManualTestSuite


class Mujoco2RvizTest(object):
    def __init__(self):
        self.process = None
        self.devnull = open(os.devnull, 'wb')

    def close_process(self):
        self.process.send_signal(subprocess.signal.SIGINT)

    def test_spawn_static_objects(self):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch'],
                                         shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)

    def test_spawn_all_objects(self):
        self.process = subprocess.Popen(['roslaunch mujoco2rviz mujoco2rviz.launch static_only:=false'],
                                         shell=True, stdout=self.devnull, stderr=subprocess.STDOUT)

    def test_clean_up_static_objects(self):
        self.close_process()

    def test_clean_up_all_objects(self):
        self.close_process()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-u", "--unattended", help="Run unattended (no user input).", action='store_true')
    args, unknown_args = parser.parse_known_args()
    rospy.init_node('mujoco2rvizl_test')
    mujoco2rviz_test = Mujoco2RvizTest()
    test_suite = ManualTestSuite(mujoco2rviz_test, unattended=args.unattended)
    test_suite.create_test('test_spawn_static_objects()', True)
    test_suite.create_test('test_clean_up_static_objects()', True)
    test_suite.create_test('test_spawn_all_objects()', True)
    test_suite.create_test('test_clean_up_all_objects()', True)
    test_suite.print_summary()

