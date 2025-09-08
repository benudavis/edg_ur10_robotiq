#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import UInt8
from cv_bridge import CvBridge

def graspStatePublisher():
    rospy.init_node('graspStatePublisher', anonymous=True)
    graspForcePub = rospy.Publisher('grasp_force', UInt8, queue_size=1)
    graspingPublisher = rospy.Publisher('is_grasping', Bool, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        try:
            
        except rospy.ROSInterruptException:
            break

        rate.sleep()


if __name__ == '__main__':
    try:
        graspStatePublisher()
    except rospy.ROSInterruptException:
        pass