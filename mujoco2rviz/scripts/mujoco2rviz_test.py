#!/usr/bin/env python

from mujoco2rviz.mujoco2marker import Mujoco2Marker
import rospy

if __name__ == '__main__':
    rospy.init_node('mujoco_to_marker', anonymous=True)
    m2m = Mujoco2Marker()
    rospy.spin()