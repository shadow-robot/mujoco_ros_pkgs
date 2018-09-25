#!/usr/bin/python

# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from mujoco_ros_msgs.srv import SpawnObjects
from object_recognition_msgs.msg import RecognizedObjectArray, RecognizedObject
from abc import abstractmethod


class SpawnSimClient(object):
    """
    Dummy class to launch Mujoco simulation with objects
    """

    def __init__(self):
        self._sim_service = rospy.ServiceProxy("mujoco/spawn_sim_environment", SpawnObjects)

    def _request_sim(self, request):
        '''
        Service client that sends request to spawn simulation
        '''
        rospy.wait_for_service("mujoco/spawn_sim_environment", 15)
        try:
            response = self._sim_service(request)
            if response.success is True:
                rospy.loginfo("Sim environment successfully spawned!")
            return response.success
        except rospy.ServiceException,   e:
            rospy.logerr("Service Call Failed:  %s" % e)

    @abstractmethod
    def _send_request(self):
        """
        Pass objects to send as request to spawn_sim_environment service
        """
        pass

if __name__ == '__main__':
    rospy.init_node("spawn_sim_client_example_node")
    spawn_sim = SpawnSimClient()
    spawn_sim._send_request()
    rospy.spin()
