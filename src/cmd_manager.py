#!/usr/bin/env python

import roslib
import rospy
import math

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

class CMD_MANAGER(object):
    def __init__(self):
        self.__pubTakeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.__pubLand = rospy.Publisher('bebop/land', Empty, queue_size=1)
        self.__pubCmdVel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=5)
        self.__pubCameraControl = rospy.Publisher('bebop/camera_control', Twist, queue_size=5)

    def send_takeoff(self):
        self.__pubTakeoff.publish(Empty())
        print "takeoff sent"

    def send_land(self):
        self.__pubLand.publish(Empty())
        print "land sent"

    def send_bebop_hover(self):
        self.send_cmd_vel(0,0,0,0)

    def send_cmd_vel(self, forward, left, up, turn_left):    # forward, left, up, turn_left: [-1,1]
        if forward < -1:
            forward = -1
        if forward > 1:
            forward = 1
        if left < -1:
            left = -1
        if left > 1:
            left = 1
        if up < -1:
            up = -1
        if up > 1:
            up = 1
        if turn_left < -1:
            turn_left = -1
        if turn_left > 1:
            turn_left = 1

        cmd_vel = Twist()
        cmd_vel.linear.x = forward
        cmd_vel.linear.y = left
        cmd_vel.linear.z = up
        cmd_vel.angular.x = 0
        cmd_vel.angular.y = 0
        cmd_vel.angular.z = -turn_left
        self.__pubCmdVel.publish(cmd_vel)

    # cf. http://bebop-autonomy.readthedocs.io/en/latest/piloting.html#moving-the-virtual-camera
    # angular.y (+)      tilt down  -> tilt up
    #           (-)      tilt up    -> tilt down
    # angular.z (+)      pan left   -> pan right
    #           (-)      pan right  -> pan left
    def set_tilt_pan(self, tilt=0, pan=0): # pan: [-35 * math.pi / 180., 35 * math.pi / 180.], tilt: [-35 * math.pi / 180., 35 * math.pi / 180.]
        if tilt < -35. * math.pi / 180.:
            tilt = -35. * math.pi / 180.
        if tilt > 35. * math.pi / 180. :
            tilt = 35. * math.pi / 180.
        if pan < -20. * math.pi / 180.:
            pan = -20. * math.pi / 180.
        if pan > 20. * math.pi / 180.:
            pan = 20. * math.pi / 180.

        camera_control = Twist()
        camera_control.angular.y = tilt * 180. / math.pi
        camera_control.angular.z = pan * 180. / math.pi
        self.__pubCameraControl.publish(camera_control)

        return pan, tilt