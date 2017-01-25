#!/usr/bin/env python

import sys
import rospy
from face_detect.srv import *
import time
import cv2
import numpy as np
from numpy import sin, cos, tan, pi
from sensor_msgs.msg import Image
from face_detect.msg import Obstacle
import roslib
from cv_bridge import CvBridge, CvBridgeError



class handle_msgs:
  def __init__(self):
    self.msg_pub = rospy.Publisher('face_detect/Obstacle_pos', Obstacle, queue_size=1)
    self.obstacle = Obstacle()
    self.rad = 0.43 * pi
    self.obstacle.r = 18
    self.obstacle.z = 0

  def posEstimate(self, window):
    x, y, h, w = window
    if x > 0:
        faceX = x + (w / 2)
        faceY = y + (h / 2)
        faceW = w
        #get liner equation from linear regression:
        #distance = 155 - 1.2 * faceW
        #get Least-squares best fit from non-linear regression:
        distance = 0.00187 * faceW * faceW - 1.17 * faceW + 183.5
        biasRad = ((faceX - 320) * self.rad) / 368
        X, Y = int(distance * cos(biasRad)), int(distance * sin(biasRad))
        if X > 0:
            self.obstacle.x, self.obstacle.y = (X, Y)
        else:
            self.obstacle.x, self.obstacle.y = (0, 0)
    else:
        self.obstacle.x, self.obstacle.y = (-1, -1)

  def sendObsPos(self):
    #print "x:%s y:%s"%(self.obstacle.x, self.obstacle.y)
    self.msg_pub.publish(self.obstacle)


class handle_images:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('bebop/image_raw',Image,self.callback)
    self.image_pub = rospy.Publisher('/image_bridge', Image, latch=False, queue_size=1)
    self.msgsHandler = handle_msgs()

  def callback(self, data):
    cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    window = self.send_to_server(0, 0, data)
    face_image = self.paint_rectangles(cv_image, window)
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(face_image, "bgr8"))
    #cv2.imshow('frame', face_image)
    #cv2.waitKey(1)

  def paint_rectangles(self, image, window):
    if window:    
      x, y, w, h = window
      self.msgsHandler.posEstimate(window)
      self.msgsHandler.sendObsPos()
      if h > 0:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0,255,0), 2)
    return image

  def send_to_server(self, para_a, para_b, frame):
    try:
        show = rospy.ServiceProxy('face_detect_server', FaceDetectSrv)
        respl = show(para_a, para_b, frame)
        return (respl.x, respl.y, respl.h, respl.w)
    except rospy.ServiceException, e:
        print 'error'


def main(args):
    rospy.wait_for_service('face_detect_server')
    rospy.init_node('obstacle_detect', anonymous=True)
    imageHandler = handle_images()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
