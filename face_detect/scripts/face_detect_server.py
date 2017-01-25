#!/usr/bin/env python

from face_detect.srv import *
import rospy
import numpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from medianFilter import median_2d_time_filter


class face_detector:
  def __init__(self):
    self.bridge = CvBridge()
    self.medianfilter = median_2d_time_filter()
    self.faceCascade = cv2.CascadeClassifier(
      "/opt/ros/kinetic/share/OpenCV-3.1.0-dev/haarcascades/haarcascade_frontalface_default.xml")

  def detect(self,image):
    faces=self.faceCascade.detectMultiScale(
        image,
        scaleFactor=1.2,
        minNeighbors=5,
        minSize=(70, 70),
        flags = cv2.CASCADE_SCALE_IMAGE
    )
    if len(faces) > 0:
        self.medianfilter.push(faces[0][0], faces[0][1], faces[0][2], faces[0][3])
        return self.medianfilter.median()
    else:
        self.medianfilter.push(0, 0, 0, 0)
        return (-1, -1, -1, -1)


faceDetect = face_detector()


def handle_face_detect(req):
    try:
        image = faceDetect.bridge.imgmsg_to_cv2(req.image,"bgr8")
        cv_image_gray=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = faceDetect.detect(cv_image_gray)
        return FaceDetectSrvResponse(faces[0], faces[1], faces[2], faces[3])


    except CvBridgeError as e:
        print e

def face_detect_server():
    rospy.init_node('face_detect_server')
    s = rospy.Service('face_detect_server', FaceDetectSrv, handle_face_detect)
    print "Ready to detect images."
    rospy.spin()

if __name__ == "__main__":
    face_detect_server()
