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
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as BS
from std_msgs.msg import Float32MultiArray as Array
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged as SS
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged as SA


class handle_msgs:
  def __init__(self):
    # initial a publisher to send Obstacle msgs to obstacle avoidence node
    self.msg_pub = rospy.Publisher('/face_detect/Obstacle_pos', Obstacle, queue_size=1)
    print 'publisher initialized'
    # set up an Obstacle object which contain the relative position and radius of the obstacle
    self.obstacle = Obstacle()
    # set the lens angle as 0.43 pi (pan)
    self.rad = 0.43 * pi
    # set the obstacle(face windows) radius as 18 cm
    self.obstacle.r = 18

  def posEstimate(self, window):
    # position = [X, Y, Z, R]
    position = []
    x, y, h, w = window
    if x > 0:
        # calculate the position and the shape of the window of the face
        faceX = x + (w / 2)
        faceY = y + (h / 2)
        faceW = w
        # calculate the distance between camera and the face
        # based on the equation from non-linear regression:
        #distance = 0.00187 * faceW * faceW - 1.17 * faceW + 183.5
        #0.00474286 x^2 - 1.838 x + 213.243
        distance = 0.00474286 * faceW * faceW - 1.838 * faceW + 213.24
        # calculate the direction of the face(represented as angle)
        biasRad = ((faceX - 320) * self.rad) / 640
        # calculate the position of the face
        if faceW > 200:
          distance = 0
        X, Y = int(distance * cos(biasRad)), int(distance * sin(biasRad))
        Z = faceY - 184
        #print 'theta:', biasRad / pi * 180, 'faceW: ', faceW
        #print 'X: ', X, 'Y: ', Y, 'theta: ', biasRad / pi * 180, 'distance: ', distance
        # make sure the position are positive numbers
        if X > 0:
            return [X, Y, Z, self.obstacle.r]
        else:
            return [0, 0, Z, self.obstacle.r]
    else:
        # if no face detected, set the position as (-1, -1, Z, R)
        return [-1, -1, -1, -1]

  def sendObsPos(self):
    # send obstacle positions
    self.msg_pub.publish(self.obstacle)


class handle_images:
  def __init__(self):
    self.bridge = CvBridge()
    # initial a subscriber to subscribe images from Bebop and send to callback function
    self.image_sub = rospy.Subscriber('/bebop/image_raw',Image,self.imgCallback, queue_size=1)
    # initial a publisher to publish images to /image_bridge
    self.image_pub = rospy.Publisher('/image_bridge', Image, latch=True, queue_size=1)
    self.battery_sub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", BS, self.batteryCallback)
    self.alertMsg_sub = rospy.Subscriber("/bebop/avoid_alert", Array, self.alertCallback)
    self.speedMsg_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", SS, self.speedCallback)
    self.msgsHandler = handle_msgs()
    self.fileSequence = 0
    self.battery = '--'
    self.font = cv2.FONT_HERSHEY_SIMPLEX
    self.alertMsg = []
    self.speed = ('0', '0', '0')
    print 'handle_images is initialized'

  def imgCallback(self, data):
    windowsList = self.send_to_server(0, 0, data)
    windows = self.msgConvert(windowsList)
    cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    #cv2.imwrite('/home/qifan/bebop_ws/src/bebop_autonomy/bebop_driver/nodes/images3/' + 
          #str(self.fileSequence) + '.jpeg', cv_image)
    #self.fileSequence += 1
    self.paint_rectangles(cv_image, windows)
    self.paint_batteryPercentage(cv_image)
    self.paint_alertMsg(cv_image)
    self.paint_speed(cv_image)
    positions = []
    for window in windows:
      points = self.msgsHandler.posEstimate(window)
      for point in points:
        positions.append(point)
    self.msgsHandler.obstacle.positions = positions
    self.msgsHandler.sendObsPos()
    #self.image_pub.publish(self.bridge.cv2_to_imgmsg(face_image, "bgr8"))
    cv2.imshow('frame', cv_image)
    cv2.waitKey(1)

  def msgConvert(self, data):
    windows = np.array(data).reshape(len(data)/4, 4)
    return windows

  def speedCallback(self, data):
    self.speed = (str(data.speedX)[:4], str(data.speedY)[:4], str(data.speedZ)[:4])

  def batteryCallback(self, data):
    self.battery = str(data.percent)

  def alertCallback(self, data):
    self.alertMsg = data.data

  def paint_batteryPercentage(self, image):
    color = (0, 255, 0)
    if self.battery != '--':
      if int(self.battery) < 70:
        color = (255, 255, 0)
      if int(self.battery) < 30:
        color = (255, 0, 0)
    cv2.putText(image, "Battery:" + self.battery, (10,50), self.font,
          0.5, color, 1, cv2.LINE_AA)

  def paint_speed(self, image):
    color = (0, 255, 0)
    cv2.putText(image, "speed X: " + self.speed[0], (300,50), self.font,
            0.5, color, 1, cv2.LINE_AA)
    cv2.putText(image, "speed Y: " + self.speed[1], (300,70), self.font,
            0.5, color, 1, cv2.LINE_AA)
    cv2.putText(image, "speed Z: " + self.speed[2], (300,90), self.font,
            0.5, color, 1, cv2.LINE_AA)

  def paint_alertMsg(self, image):
    color = (0, 0, 255)
    if self.alertMsg:
      cv2.putText(image, "Pushback:" + str(self.alertMsg[0])[:5], (100,50), self.font,
            0.5, color, 1, cv2.LINE_AA)
      cv2.putText(image, "Turn:" + str(self.alertMsg[1])[:5], (100,70), self.font,
            0.5, color, 1, cv2.LINE_AA)
      cv2.putText(image, "AutoLevel:" + str(self.alertMsg[2])[:5], (100,90), self.font,
            0.5, color, 1, cv2.LINE_AA)

  def paint_rectangles(self, image, windows):
    for window in windows:
      x, y, w, h = window
      if h > 0:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0,255,0), 2)

  def send_to_server(self, para_a, para_b, frame):
    # send ROS Image msgs to face detection server
    try:
        show = rospy.ServiceProxy('/face_detect/facedetectServer', FaceDetectSrv)
        respl = show(para_a, para_b, frame)
        return respl.windows
    except rospy.ServiceException, e:
        print 'error'


def main(args):
    rospy.init_node('obstacle_detect', anonymous=True)
    print 'obstacleDetect.py is launched'
    rospy.wait_for_service('/face_detect/facedetectServer')
    print 'service detected'
    imageHandler = handle_images()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
