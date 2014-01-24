#!/usr/bin/env python
# encoding: utf-8



"""
joy2twist.py - Provides Twist messages given a joy topic.

Created by William Woodall on 2010-07-12.
"""
__author__ = "William Woodall"

###  Imports  ###

# ROS imports
import roslib; roslib.load_manifest('human_interface')
import rospy

# ROS msg and srv imports
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Python Libraries
import sys
import traceback

###  Variables  ###
LINEAR_SPEED = 0.4
ANGULAR_SPEED = 0.8

###  Classes  ###

class Joy2Twist(object):
    joyEnabled = True

    """Joy2Twist ROS Node"""
    def __init__(self):
        # Initialize the Node
        rospy.init_node("Joy2Twist")
        
        # Setup the Joy topic subscription
        self.joy_subscriber = rospy.Subscriber("joy", Joy, self.handleJoyMessage, queue_size=1)
        
        # Setup the Twist topic publisher
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist)
        
        # Spin
        rospy.spin()
    
    def handleJoyMessage(self, data):
        """Handles incoming Joy messages"""
        msg = Twist()
        msg.linear.x = data.axes[1] * LINEAR_SPEED
        
        msg.angular.z = data.axes[2] * ANGULAR_SPEED
        if data.buttons[5]==1:
            msg.linear.x *= 2;
            msg.angular.z *= 1.5;
 
        if data.buttons[0]==1:
            self.joyEnabled = True;
    
        if data.buttons[1]==1:
            self.joyEnabled = False;
       
        if self.joyEnabled:
            self.twist_publisher.publish(msg)
    

###  If Main  ###
if __name__ == '__main__':
    try:
        Joy2Twist()
    except:
        rospy.logerr("Unhandled Exception in the joy2Twist Node:+\n"+traceback.format_exc())
