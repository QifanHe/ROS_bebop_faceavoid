#!/usr/bin/env python

import sys
import rospy
from face_detect.srv import *
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
import roslib
from cv_bridge import CvBridge, CvBridgeError

cap = cv2.VideoCapture(0)
bridge = CvBridge()


def send(x, y):
    frame = cap.read()[1]
    #cv2.imshow('as', frame)
    #cv2.waitKey(1)
    i = bridge.cv2_to_imgmsg(frame, "bgr8")
    try:
        show = rospy.ServiceProxy('face_detect_server', FaceDetectSrv)
        respl = show(3, 4, i)
        print respl.x, respl.y, respl.h, respl.w
    except rospy.ServiceException, e:
        print 'error'


if __name__ == "__main__":
    rospy.wait_for_service('face_detect_server')
    t = time.time()
    for count in range(90):
        send(count, count)
    print time.time() - t