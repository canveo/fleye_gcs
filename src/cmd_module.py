#!/usr/bin/env python

import roslib
roslib.load_manifest('bebop_msgs')
import rospy
# import numpy as np
# import math

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
# from sensor_msgs.msg import CompressedImage        # for receiving the video feed
# from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
# from drone_status import DroneStatus

# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import String
# from std_msgs.msg import Float32
# from tum_ardrone.msg._filter_state import filter_state
# from tum_ardrone.msg._autopilot_cmd import autopilot_cmd
# from tum_ardrone.msg._tagged_objects_state import tagged_objects_state

# this class send control commands to bebop/takeoff, land, cmd_vel and camera_control

class CMD_MODULE(object):
    def __init__(self):
        self.pubTakeoff = rospy.Publisher('bebop/takeoff', Empty)
        self.pubLand = rospy.Publisher('bebop/land', Empty)
        self.pubCmdVel = rospy.Publisher('bebop/cmd_vel', Twist)
        self.pubCameraControl = rospy.Publisher('bebop/camera_control', Twist)

    def send_takeoff(self):
        self.pubTakeoff.publish(Empty())
        print "takeoff sent"

    def send_land(self):
        self.pubLand.puslish(Empty())
        print "land sent"

    def send_bebop_hover(self):
        self.send_cmd_vel(0,0,0,0)

    def send_cmd_vel(self, forward, left, up, turn_left):    # forward, left, up, turn_left: [-1,1]
        is_invalid = False
        if not -1 <= forward <= 1:
            print "invalid forward velocity:", forward, "not in range [-1, 1]"
            is_invalid = True
        if not -1 <= left <= 1:
            print "invalid left velocity:", left, "not in range [-1, 1]"
            is_invalid = True
        if not -1 <= up <= 1:
            print "invalid up velocity:", up, "not in range [-1, 1]"
            is_invalid = True
        if not -1 <= turn_left <= 1:
            print "invalid turn_left velocity:", turn_left, "not in range [-1. 1]"
        if is_invalid: return

        cmd_vel = Twist()
        cmd_vel.linear.x = forward
        cmd_vel.linear.y = left
        cmd_vel.linear.z = up
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = -turn_left
        self.pubCmdVel.publish(cmd_vel)

    def send_tilt_pan(self, tilt, pan=0): # pan: [-35, 35], tilt: [-70, 35]
        is_invalid = False
        if not -70 <= tilt <= 35:
            print "invalid tilt angle:", tilt, "not in range [-70. 35]"
            is_invalid = True
        if not -35 <= pan <= 35:
            print "invalid pan angle:", pan, "not in range [-35, 35]"
            is_invalid = True
        if is_invalid: return

        camera_control = Twist()
        camera_control.angular.y = tilt
        camera_control.angular.z = pan
        self.pubCameraControl.publish(camera_control)

