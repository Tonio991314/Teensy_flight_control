#!/usr/bin/env python

from __future__ import print_function

import threading
import sys
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

important=["I","O","L","a", "d", "s","w", "Q", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0"]

def publisher():
    pub = rospy.Publisher('/publisher_keyboard', String, queue_size=10)
    rate=rospy.Rate(66)
    # print("type:", type(key) )
    # while not rospy.is_shutdown():
    start=2
    while not rospy.is_shutdown():
        print("Please input a key:")
        keys = getKey()
        if keys=="\x03":
            print("Exit code xD")
            sys.exit()
        elif keys in important:
            pub.publish(keys)
            time.sleep(0.01)
        else:
            pub.publish(keys)
        rate.sleep()

import sys
import termios
import tty
import select

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = "0"
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def getKey_():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [])
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(key)
    return key

if __name__=="__main__":
    rospy.init_node('keyboard')
    settings = termios.tcgetattr(sys.stdin)    
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('/publisher_keyboard', String, queue_size=10)
    publisher()
    # key = getKey()
    # try:
    #     publisher(key)
    # except rospy.ROSInterruptException:
    #     pass

    # rospy.spin()
    

