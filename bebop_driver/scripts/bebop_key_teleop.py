#!/usr/bin/env python
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty
import sys
import curses


twist = Twist()
stdscr = curses.initscr()
curses.noecho()
stdscr.keypad(1)
takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
land = rospy.Publisher('/bebop/land', Empty, queue_size=1)
pub = rospy.Publisher('/bebop/keyBoard', Twist, queue_size=1)
camctrl = rospy.Publisher('/bebop/camera_control', Twist, queue_size=1)
rospy.init_node('bebop_key', anonymous=True)


def keyboard(key):
    switcher = {
    't' : 'TAKEOFF',
    'l' : 'LANDING',
    'a' : 'LEFT',
    'w' : 'FORWARD',
    'd' : 'RIGHT',
    's' : 'BACK',
    'q' : 'LEFT-ROTATION',
    'e' : 'RIGHT-ROTATION',
    'z' : 'UP',
    'x' : 'DOWN',
    '=' : 'GAIN SPEED',
    '-' : 'REDUCE SPEED',
    ']' : 'GAIN ROTATION SPEED',
    '[' : 'REDUCE ROTATION SPEED',
    '.' : 'GAIN UP/DOWN SPEED',
    ',' : 'REDUCE UP/DOWN SPEED',
    curses.KEY_UP : 'CAMERA UP',
    curses.KEY_DOWN : 'CAMERA DOWN',
    curses.KEY_RIGHT : 'CAMERA RIGHT',
    curses.KEY_LEFT : 'CAMERA LEFT'
    }
    if key in range(256):
        if chr(key) in switcher:
            return switcher[chr(key)]
    else:
        return switcher[key]



class parameter:

    def __init__(self):
        self.speed, self.r_speed, self.ud_speed = 0.2, 0.2, 0.2
        self.cam_tilt, self.cam_pan = 0, 0

    def speed_gain(self):
        if round(self.speed, 3) < 1:
            self.speed += 0.1

    def speed_reduce(self):
        if round(self.speed, 3) > 0.1:
            self.speed -= 0.1

    def rotation_speed_gain(self):
        if round(self.r_speed, 3) < 1:
            self.r_speed += 0.1

    def rotation_speed_reduce(self):
        if round(self.r_speed, 3) > 0.1:
            self.r_speed -= 0.1

    def up_down_speed_gain(self):
        if round(self.ud_speed, 3) < 1:
            self.ud_speed += 0.1

    def up_down_speed_reduce(self):
        if round(self.ud_speed, 3) > 0.1:
            self.ud_speed -= 0.1

    def cam_tilt_up(self):
        if round(self.cam_tilt, 3) < 50:
            self.cam_tilt += 1

    def cam_tilt_down(self):
        if round(self.cam_tilt, 3) > -50:
            self.cam_tilt -= 1

    def cam_pan_right(self):
        if round(self.cam_pan, 3) < 80:
            self.cam_pan += 1

    def cam_pan_left(self):
        if round(self.cam_pan, 3) > -80:
            self.cam_pan -= 1

para = parameter()


def getTwist(direction):
    switcher = {'LEFT' : (0, round(para.speed, 3), 0, 0, 0),
    'RIGHT' : (0, -round(para.speed, 3), 0, 0, 0),
    'FORWARD' : (round(para.speed, 3), 0, 0, 0, 0),
    'BACK' : (-round(para.speed, 3), 0, 0, 0, 0),
    'UP' : (0, 0, round(para.ud_speed, 3), 0, 0),
    'DOWN' : (0, 0, -round(para.ud_speed, 3), 0, 0),
    'LEFT-ROTATION' : (0, 0, 0, 0, round(para.r_speed, 3)),
    'RIGHT-ROTATION' : (0, 0, 0, 0, -round(para.r_speed, 3)),
    'CAM' : (0, 0, 0, round(para.cam_tilt, 3), round(para.cam_pan, 3))}
    if direction in switcher:
        val = switcher[direction]
    else:
        val = (0, 0, 0, 0, 0)
    twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.y, twist.angular.z = val
    return twist


def cmdPublish(command):
    cmd = command
    if cmd == 'TAKEOFF':
        takeoff.publish()
    elif cmd == 'LANDING':
        land.publish()
    elif cmd == 'GAIN SPEED':
        para.speed_gain()
    elif cmd == 'REDUCE SPEED':
        para.speed_reduce()
    elif cmd == 'GAIN ROTATION SPEED':
        para.rotation_speed_gain()
    elif cmd == 'REDUCE ROTATION SPEED':
        para.rotation_speed_reduce()
    elif cmd == 'GAIN UP/DOWN SPEED':
        para.up_down_speed_gain()
    elif cmd == 'REDUCE UP/DOWN SPEED':
        para.up_down_speed_reduce()
    elif cmd == 'CAMERA UP':
        para.cam_tilt_up()
        camctrl.publish(getTwist('CAM'))
    elif cmd == 'CAMERA DOWN':
        para.cam_tilt_down()
        camctrl.publish(getTwist('CAM'))
    elif cmd == 'CAMERA RIGHT':
        para.cam_pan_right()
        camctrl.publish(getTwist('CAM'))
    elif cmd == 'CAMERA LEFT':
        para.cam_pan_left()
        camctrl.publish(getTwist('CAM'))
    else:
        MSG = getTwist(cmd)
        pub.publish(MSG)


def scrInitial():
    stdscr.addstr(0, 15, '=====Keyboard Controller for BEBOP DRONE=====')
    stdscr.addstr(1, 0, '-----------------------------------------------------------------------')
    stdscr.addstr(2, 0, '   TAKEOFF(T)    LANDING(L)        Use ARROW KEYS to control the camera')
    stdscr.addstr(3, 0, '-----------------------------------------------------------------------')
    stdscr.addstr(4, 0, '   LEFT-ROTATION(Q)       RIGHT-ROTATION(E)            |         ')
    stdscr.addstr(5, 0, '                  FORWARD (W)                          |      UP(Z)           ')
    stdscr.addstr(6, 0, '   LEFT(A)         BACKWARD(S)       RIGHT(D)          |      DOWN(X)         ')
    stdscr.addstr(7, 0, '-----------------------------------------------------------------------')
    stdscr.addstr(8, 0, '  HORIZONTAL SPEED            VERTICAL SPEED             ROTATION SPEED')
    stdscr.addstr(9, 0, '     (-)' + str(para.speed) + '(+)                   (<)'
                    + str(para.ud_speed) + '(>)                  ([)'
                    + str(para.r_speed) + '(])')
    stdscr.addstr(11, 3, '                                                      SHIFT+!   to quit.' )
    stdscr.refresh()


def screenPrint(command):
    try:
        pad = curses.newpad(100, 100)
        pad.addstr(5,20,'sending:'+command)
        pad.refresh(0,0, 12,5, 20,75)
        scrInitial()
    except:
        pass


def main(args):
    scrInitial()
    user_input = ''
    while user_input != ord('!'):
        user_input = stdscr.getch()
        command = keyboard(user_input)
        cmdPublish(command)
        screenPrint(command)
    curses.endwin()

    
if __name__ == '__main__':
    main(sys.argv)
