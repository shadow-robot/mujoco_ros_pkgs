#!/usr/bin/env python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

# Example use: rosrun sr_utl5 pick_place_client.py -o "duplo_2x4x1_0"  -p 0.761 0.873 0.7633 0.0 0.0 1.5707963267

import rospy
import tf
import argparse
from geometry_msgs.msg import Pose
from sr_msgs_common.srv import MoveObject
from sr_manipulation_grasp_conductor.pick_place_client import request_pick_and_place


if __name__ == '__main__':
    rospy.init_node('mating_map_mock_client', anonymous=True)
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--object_name', type=str, help='Name of the duplo block frame')
    parser.add_argument('-p', '--place_position', nargs=6, help='Pose to place object to')
    args = parser.parse_args()

    place_position = [float(val) for val in args.place_position]
    chosen_object = args.object_name

    rospy.loginfo("Chose {} to be picked".format(chosen_object))
    rospy.loginfo("Moving object to position: {}".format(place_position))

    result = request_pick_and_place(chosen_object, place_position)
    if result:
        rospy.loginfo("Successfully moved object")
