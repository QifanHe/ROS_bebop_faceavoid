#!/usr/bin/env python

from face_detect.srv import *
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from medianFilter import median_2d_time_filter


class face_detector:
  def __init__(self):
    self.bridge = CvBridge()
    self.faceCascade = cv2.CascadeClassifier(
      "/opt/ros/kinetic/share/OpenCV-3.2.0-dev/haarcascades/haarcascade_frontalface_default.xml")
    self.median = median_2d_time_filter()
    self.process = True

  def detect(self,image):
    faces=self.faceCascade.detectMultiScale(
        image,
        scaleFactor=1.4,
        minNeighbors=3,
        minSize=(50, 50),
        flags = cv2.CASCADE_SCALE_IMAGE
    )
    if type(faces) == tuple:
        self.median.push(-1, -1, -1, -1)
        return 0
    self.median.push(faces[0][0], faces[0][1], faces[0][2], faces[0][3])

faceDetect = face_detector()


def handle_face_detect(req):
    try:
        if faceDetect.process == True:
            image = faceDetect.bridge.imgmsg_to_cv2(req.image,"bgr8")
            cv_image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #else:
        #    faceDetect.process = True
            #print cv_image_gray[50][50]
            faceDetect.detect(cv_image_gray)
            faceVal = faceDetect.median.median()
            faceMsg = []
            for val in faceVal:
                faceMsg.append(val)
            return FaceDetectSrvResponse(faceMsg)
    except CvBridgeError as e:
        print e

def face_detect_server():
    rospy.init_node('facedetectServer')
    s = rospy.Service('facedetectServer', FaceDetectSrv, handle_face_detect)
    print "Ready to detect images."
    rospy.spin()

if __name__ == "__main__":
    face_detect_server()
