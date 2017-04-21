#!/usr/bin/env python
import rospy
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged as BS
#from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged as SS
#from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged as SA
import time
import numpy as np
from numpy import cos, sin, pi
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import roslib
import cv2
import sys
import json

"""
class Position:
    def __init__(self, x = 0, y = 0, z = 0):
        self.pos = np.array([x, y, z], dtype='float')
        self.angle = 0
        plt.ion()

    def updateLinear(self, data):
        x, y, z = data.speedX, data.speedY, data.speedZ
        new = np.array([x * cos(self.angle) + y * sin(self.angle),
            y * cos(self.angle) + x * sin(self.angle), z], dtype='float')
        self.pos += new * 0.2
        self.X = self.pos[0]
        self.Y = self.pos[1]
        self.Z = self.pos[2]
    
    def updateAngular(self, data):
        self.angle = data.yaw


    def showPos(self):
        results = {"X": self.X, "Y": self.Y, "Z": self.Z, "D": self.angle}
        print json.dumps(results, sort_keys=True)
"""      
"""
pos = Position()

def callLinear(data):
    pos.updateLinear(data)
    pos.showPos()
def callAngular(data):
    pos.updateAngular(data)
"""

def callback(data):
    print data.percent

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", BS, callback)
    #rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", SS, callLinear)
    rospy.spin()

if __name__ == '__main__':
    listener()

