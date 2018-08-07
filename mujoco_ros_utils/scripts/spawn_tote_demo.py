#!/usr/bin/python

# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from mujoco_ros_msgs.srv import SpawnObjects
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
from mujoco_ros_utils.spawn_sim_client import SpawnSimClient


class ToteDemoSpawnSimClient(SpawnSimClient):

    def __init__(self):
        super(ToteDemoSpawnSimClient, self).__init__()

    def _send_request(self):
        request = RecognizedObjectArray()

        rec_object_1 = RecognizedObject()
        rec_object_1.type.key = "utl5_medium"
        rec_object_1.pose.pose.pose.position.x = 0.4
        rec_object_1.pose.pose.pose.position.y = 0.7
        rec_object_1.pose.pose.pose.position.z = 0
        rec_object_1.pose.pose.pose.orientation.x = 0
        rec_object_1.pose.pose.pose.orientation.y = 0
        rec_object_1.pose.pose.pose.orientation.z = 0
        rec_object_1.pose.pose.pose.orientation.w = 1

        request.objects = [rec_object_1]
        super(ToteDemoSpawnSimClient, self)._request_sim(request)

if __name__ == '__main__':
    rospy.init_node("spawn_sim_client_example_node")
    spawn_sim =  ToteDemoSpawnSimClient()
    spawn_sim._send_request()
    rospy.spin()