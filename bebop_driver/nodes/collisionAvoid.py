#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from face_detect.msg import Obstacle
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
import numpy as np
from numpy import sin, cos, arctan
import sys
from math import sqrt
import time
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged as SS


class Avoid:

  def __init__(self):
    # initial publishers to send command to Bebop
    self.avoid_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    self.takeoff_pub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    self.land_pub = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    # initial a publisher to send avoidance message
    self.avoidAlert_pub = rospy.Publisher('/bebop/avoid_alert', Float32MultiArray, queue_size=1)
    # initial a list to contain avoidance forces
    self.f = []

  def avoid(self):
    # add avoidance force to piloting values
    # for example, Fx means user control, fx means avoidance force, FX means joint force
    twist = Twist()
    array = Float32MultiArray()
    Fx, Fy, Fz = self.pilotMsg[0], self.pilotMsg[1], self.pilotMsg[2]
    fx, fy, fz = self.f[0], self.f[1], self.f[2]
    jointFX = Fx - Fx * fx
    jointFY = Fx * fy
    # if no vertical piloting control, use auto-hold altitude control
    if Fz == 0:
      jointFZ = fz
    else:
      jointFZ = Fz
    self.pilotMsg[0], self.pilotMsg[1], self.pilotMsg[2] = jointFX, jointFY, jointFZ
    #print 'fy:', fy
    #print 'jointFY: ', jointFY
    piloting = self.pilotMsg.tolist()
    array.data = self.f
    self.avoidAlert_pub.publish(array)
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.y, twist.angular.z = piloting
    self.avoid_pub.publish(twist)
    #print twist

  def takeoff(self):
    self.takeoff_pub.publish()

  def land(self):
    self.land_pub.publish()

  def altitudeHold(self):
    # hold the altitude
    twist = Twist()
    piloting = (0, 0, self.f[2], 0, 0)
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.y, twist.angular.z = piloting
    self.avoid_pub.publish(twist)


class msgListener:

  def __init__(self):
    self.avoid = Avoid()
    # initial subscribers
    # receive obstacle positions
    self.obs_sub = rospy.Subscriber('/face_detect/Obstacle_pos', Obstacle, self.callObsHandler)
    # receive piloting msgs
    self.key_sub = rospy.Subscriber('/bebop/keyBoard', Twist, self.callKeyHandler)
    # receive takeoff msg
    self.takeoff_sub = rospy.Subscriber('/bebop/keyBoard/takeoff', Empty, self.callTakeoffHandler)
    # receive landing msg
    self.land_sub = rospy.Subscriber('/bebop/keyBoard/land', Empty, self.callLandHandler)
    self.speed_sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", SS, self.callSpeed)
    self.obsPos = Obstacle()
    # set a tookoff flag
    self.tookoff = False
    # set an altitude auot-hold flag
    self.hold = True
    self.time = 0
    self.speed = (0, 0, 0)

  def callObsHandler(self, data):
    # convert obstacle msgs to python list
    self.obstacles = np.array(data.positions).reshape(len(data.positions)/4, 4).tolist()
    self.forceGenerator()
    # if auto-altitude is switched on and no keyboard input over 0.5s, start altitude auto-hold
    if self.hold and self.tookoff and time.time() - self.time > 0.5:
      self.avoid.altitudeHold()

  def callKeyHandler(self, data):
    # if keyboard input detected, update self.time
    self.time = time.time()
    self.key_msg = data
    piloting = [data.linear.x, data.linear.y, data.linear.z, data.angular.y, data.angular.z]
    self.avoid.pilotMsg = np.array(piloting)
    # if obstacle detected, the keyboard control would be tookover by the program

    if self.obstacles[0][0] > -1:
        self.takeOver()
    else:
        self.forward()

  def callTakeoffHandler(self, data):
    self.tookoff = True
    self.avoid.takeoff()

  def callLandHandler(self, data):
    self.tookoff = False
    self.avoid.land()

  def callSpeed(self, data):
    self.speed = (data.speedX, data.speedY, data.speedZ)

  def takeOver(self):
    # if the user control the drone forward a face, start face-avoidance
    if self.key_msg.linear.x > 0:
      self.avoid.avoid()
    else:
      self.forward()

  def forward(self):
    # forward keyboard control to Bebop
    self.avoid.avoid_pub.publish(self.key_msg)

  def forceGenerator(self):
    fx, fy, fz, x, y, z = 0, 0, 0, 0.0, 0.0, 0.0
    # calculate avoidance or auto-altitude force based on x, y and z
    for obstacle in self.obstacles:
      x, y, z, r = obstacle
      # calculate fx
      if x < 40:
        fx = 2
      else:
        fx = (104 / float(x) - 0.6) * (100 - 2.5 * abs(float(y))) / 50
      # multiplied by a speedX factor
      fx = fx * ((1 + abs(self.speed[0])) ** 2)
      # if back speed more than 0.3 m/s, no need to push back.
      if self.speed[0] > 0.2:
        fx = 0
      # calculate fy
      if x < 50:
        fy = 1
      else:
        if y > 0:
          #fy = -(21.8 / (float(x) - 48.5) - 0.3) * (-abs(float(y))/50 + 1)
          fy = (float(x) / (-130) + 1.27) * (-abs(float(y))/50 + 1)
        else:
          #fy = (21.8 / (float(x) - 48.5) - 0.3) * (-abs(float(y))/50 + 1)
          fy = -(float(x) / (-130) + 1.27) * (-abs(float(y))/50 + 1)
      # calculate fz
      if abs(z) > 10:
        if z > 0:
          fz = -0.2
        else:
          fz = 0.2
      elif abs(z) > 5:
        if z > 0:
          fz = -0.1
        else:
          fz = 0.1
      else:
          fz = 0
    #print "fx = ", fx, "fy = ", fy, 'x, y: ', x, y
    self.avoid.f = [fx, fy, fz, x, y]


if __name__ == "__main__":
    rospy.init_node('obstacle_detect', anonymous=True)
    msgListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()
