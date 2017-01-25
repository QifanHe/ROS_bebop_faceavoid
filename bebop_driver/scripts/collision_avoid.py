#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from face_detect.msg import Obstacle
from std_msgs.msg import String
import sys
from math import sqrt


class Avoid:

  def __init__(self):
    self.avoid_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
    self.cmdSwitcher = {'LEFT' : (0, 0.2, 0, 0, 0),
    'RIGHT' : (0, 0.2, 0, 0, 0),
    'BACK' : (0.2, 0, 0, 0, 0),
    'UP' : (0, 0, 0.2, 0, 0),
    'DOWN' : (0, 0, 0.2, 0, 0),
    'LEFT-ROTATION' : (0, 0, 0, 0, 0.2),
    'RIGHT-ROTATION' : (0, 0, 0, 0, 0.2)}
  def twistGenerator(self, cmd):
    twist = Twist()
    if cmd in self.cmdSwitcher:
        val = self.cmdSwitcher[cmd]
    else:
        val = (0, 0, 0, 0, 0)
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.y, twist.angular.z = val
    return twist
  def shiftRight(self):
    print 'shiftRight'
    self.avoid_pub.publish(self.twistGenerator('RIGHT'))
  def shiftLeft(self):
    print 'shiftLeft'
    self.avoid_pub.publish(self.twistGenerator('LEFT'))
  def holdBack(self):
    print 'holdBack'
    self.avoid_pub.publish(self.twistGenerator('BACK'))
  def slowDown(self):
    pass

class msgListener:

  def __init__(self):
    self.avoid = Avoid()
    self.obs_sub = rospy.Subscriber('/face_detect/Obstacle_pos', Obstacle, self.callObsHandler)
    self.key_sub = rospy.Subscriber('/bebop/keyBoard', Twist, self.callKeyHandler)
    self.key_msg = Twist()
    self.obsPos = Obstacle()
    self.level = 0
  def callObsHandler(self, data):
    self.obsPos.x, self.obsPos.y, self.obsPos.z, self.obsPos.r = (data.x, data.y, data.z, data.r)
    self.setLevel()

  def takeOver(self):
    if self.level == 3:
        self.avoid.holdBack()
    elif self.level == 1:
        self.forward()
    elif self.level == 0:
        self.forward()
    else:
        if self.key_msg.linear.x > 0:
            if self.obsPos.y > 0:
                self.avoid.shiftLeft()
            else:
                self.avoid.shiftRight()

  def forward(self):
    self.avoid.avoid_pub.publish(self.key_msg)

  def callKeyHandler(self, data):
    self.key_msg = data
    if self.obsPos.x > 0:
        self.takeOver()
    else:
        self.forward()

  def setLevel(self):
    levelSwitcher = {
        0 : range(100, 1000),
        1 : range(70, 100),
        2 : range(50, 70),
        3 : range(0, 50)}
    X, Y, R = self.obsPos.x, self.obsPos.y, self.obsPos.r
    distance = int(sqrt(X * X + Y * Y))
    #print 'distance:', distance
    for key, value in levelSwitcher.items():
        if distance in value:
            self.level = key


if __name__ == "__main__":
    rospy.init_node('obstacle_detect', anonymous=True)
    msgListener()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
        cv2.destroyAllWindows()
