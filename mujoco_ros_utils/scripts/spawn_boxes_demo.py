#!/usr/bin/python

# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from mujoco_ros_msgs.srv import SpawnObjects
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
from mujoco_ros_utils.spawn_sim_client import SpawnSimClient


class ExampleSpawnSimClient(SpawnSimClient):
    """
    Dummy class to launch Mujoco simulation with objects
    """

    def __init__(self):
        super(ExampleSpawnSimClient, self).__init__()

    def _send_request(self):
        request = RecognizedObjectArray()

        # Dummy obj 1
        rec_object_1 = RecognizedObject()
        rec_object_1.type.key = "box"
        rec_object_1.pose.pose.pose.position.x = 0.3
        rec_object_1.pose.pose.pose.position.y = 0.7
        rec_object_1.pose.pose.pose.position.z = 0
        rec_object_1.pose.pose.pose.orientation.x = 0
        rec_object_1.pose.pose.pose.orientation.y = 0
        rec_object_1.pose.pose.pose.orientation.z = 0
        rec_object_1.pose.pose.pose.orientation.w = 1

        # Dummy obj 2
        rec_object_2 = RecognizedObject()
        rec_object_2.type.key = "box"
        rec_object_2.pose.pose.pose.position.x = 0.3
        rec_object_2.pose.pose.pose.position.y = 1
        rec_object_2.pose.pose.pose.position.z = 0
        rec_object_2.pose.pose.pose.orientation.x = 0.707
        rec_object_2.pose.pose.pose.orientation.y = 0
        rec_object_2.pose.pose.pose.orientation.z = 0
        rec_object_2.pose.pose.pose.orientation.w = 0.707
        request.objects = [rec_object_1, rec_object_2]
        self._request_sim(request)


if __name__ == '__main__':
    rospy.init_node("spawn_sim_client_example_node")
    spawn_sim = ExampleSpawnSimClient()
    spawn_sim._send_request()
    rospy.spin()
