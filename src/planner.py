#!/usr/bin/env python

import roslib

import rospy

from std_msgs.msg import Header, Empty, Float32, Float32MultiArray, String, Int32
from sensor_msgs.msg import Image, CompressedImage, ChannelFloat32, PointCloud
from geometry_msgs.msg import Pose, PoseStamped                                     #http://docs.ros.org/api/geometry_msgs/html/index-msg.html

from tf import transformations

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

from target import Target

from constant import *

class PLANNER(object):
    def __init__(self):
        self.__hover_position = None    # format: [x_world, y_world, z_world]

        # debug
        self.__pub_hover_x = rospy.Publisher('fleye/debug/hover_position_x', Float32, queue_size=1)
        self.__pub_hover_y = rospy.Publisher('fleye/debug/hover_position_y', Float32, queue_size=1)
        self.__pub_hover_z = rospy.Publisher('fleye/debug/hover_position_z', Float32, queue_size=1)

    def set_hover_position(self, position):
        self.__hover_position = position

    def get_hover_position(self):
        return self.__hover_position

    # def update_hover_position(self, position_offset):
    #     self.__hover_position[0] += position_offset[0]
    #     self.__hover_position[1] += position_offset[1]
    #     self.__hover_position[2] += position_offset[2]

    def pub_debug_info(self):
        if self.__hover_position is not None:
            self.__pub_hover_x.publish(self.__hover_position[0])
            self.__pub_hover_y.publish(self.__hover_position[1])
            self.__pub_hover_z.publish(self.__hover_position[2])