#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

FRAME_SIZE = (640, 480)

def tactileFingertipPublisher():
    rospy.init_node('tactile_fingertip_publisher', anonymous=True)
    tactile1_pub = rospy.Publisher('tactile1', Image, queue_size=1)
    tactile2_pub = rospy.Publisher('tactile2', Image, queue_size=1)

    bridge = CvBridge()
    rate = rospy.Rate(30)

    tactile1_path = "/dev/v4l/by-id/usb-tactile1_tactile1_0123456789-video-index0"
    tactile2_path = "/dev/v4l/by-id/usb-tactile2_tactile2_0123456789-video-index0"

    cap1 = cv2.VideoCapture(tactile1_path, cv2.CAP_V4L2)
    cap2 = cv2.VideoCapture(tactile2_path, cv2.CAP_V4L2)

    # Note: I highly recommend adding the MJPG format flag here if you hit bandwidth limits
    cap1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap2.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    cap1.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
    cap1.set(cv2.CAP_PROP_FPS, 30)
    cap1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_SIZE[0])
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_SIZE[1])
    cap2.set(cv2.CAP_PROP_FPS, 30)
    cap2.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # 1. The TRY block now wraps the ENTIRE loop
    try:
        while not rospy.is_shutdown():
            ret1, frame1 = cap1.read()
            ret2, frame2 = cap2.read()

            if ret1:
                try:
                    image_msg1 = bridge.cv2_to_imgmsg(frame1, encoding="bgr8")
                    image_msg1.header.stamp = rospy.Time.now()
                    image_msg1.header.frame_id = "tactile1"
                    tactile1_pub.publish(image_msg1)
                except CvBridgeError as e:
                    rospy.logerr("CvBridgeError 1: %s", e)

            if ret2:
                try:
                    image_msg2 = bridge.cv2_to_imgmsg(frame2, encoding="bgr8")
                    image_msg2.header.stamp = rospy.Time.now()
                    image_msg2.header.frame_id = "tactile2"
                    tactile2_pub.publish(image_msg2)
                except CvBridgeError as e:
                    rospy.logerr("CvBridgeError 2: %s", e)

            # 2. Add waitKey(1) so the imshow window actually updates
            if ret1 and ret2:
                combined = cv2.hconcat([frame1, frame2])
                cv2.imshow("tactile", combined)
                cv2.waitKey(1) 

            rate.sleep()

    # 3. The FINALLY block only executes when the node is shutting down
    finally:
        rospy.loginfo("Shutting down node, releasing cameras...")
        if cap1:
            cap1.release()
        if cap2:
            cap2.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tactileFingertipPublisher()
    except rospy.ROSInterruptException:
        pass