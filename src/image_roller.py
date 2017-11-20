#!/usr/bin/env python

import roslib

import rospy
import cv2
import math

import os

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from threading import Lock

import numpy as np


# SET -> COMPUTE -> GET
class IMAGE_ROLLER(object):
    def __init__(self):

        self.bridge = CvBridge()

        self.__orb_roll = None              # format: in radians

        self.__target_roll = None              # in radians

        self.__control_goal_roll = 0             # in radians


        # uncompressed image for roll
        rospy.Subscriber('/bebop/image_raw', Image, self.image_callback, queue_size=1)
        self.__pub_rolled_image = rospy.Publisher('bebop/rolled_image_raw', Image, queue_size=1)
        # self.__pub_snap_shot_on_board = rospy.Publisher('bebop/snapshot', Empty, queue_size=1)
        self.__image_buffer = None
        self.__image_buffer_lock = Lock()

        self.__target_1_composition = None      # format: [u, v, 1] in pixel
        self.__target_2_composition = None

        self.__target_position = None # [x,y,z]
        self.__current_position = None # [x,y,z]


        self.__target_orientation = None
        self.__current_orientation = None

        # debug
        self.__pub_current_roll = rospy.Publisher('fleye/debug/current_image_roll', Float32, queue_size=1)
        self.__pub_target_roll = rospy.Publisher('fleye/debug/target_image_roll', Float32, queue_size=1)

        # save image
        self.__counter = 0
        self.__counter_MAX = 30
        rospy.Subscriber('/bebop/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.__cmd_vel_buffer = None
        self.__cmd_vel_lock = Lock()


    def cmd_vel_callback(self,data):
        self.__cmd_vel_lock.acquire()
        self.__cmd_vel_buffer = data
        self.__cmd_vel_lock.release()

    def image_callback(self,data):
        # self.__image_buffer_lock.acquire()
        self.__image_buffer = data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            (rows,cols,channels) = cv_image.shape
            mat_roll = cv2.getRotationMatrix2D((cols/2, rows/2), self.__control_goal_roll / math.pi * 180., 1)
            dst = cv2.warpAffine(cv_image,mat_roll,(cols,rows))
            if self.__target_1_composition is not None:
                cv2.circle(dst, (self.__target_1_composition[0],self.__target_1_composition[1]), 1, (255,0,0))
            if self.__target_2_composition is not None:
                cv2.circle(dst, (self.__target_2_composition[0],self.__target_2_composition[1]), 1, (0,0,255))

            circle_img = np.zeros((rows,cols), np.uint8)
            cv2.circle(circle_img,(cols/2,rows/2),rows/2,1,thickness=-1)
            masked_data = cv2.bitwise_and(dst, dst, mask=circle_img)

            # publish for visualization
            self.__pub_rolled_image.publish(self.bridge.cv2_to_imgmsg(masked_data, "bgr8"))


            # save
            if self.__target_1_composition is None or self.__target_2_composition is None:
                return
            else:
                print 'img_counter', self.__counter
            dir = "/home/ziquan/img_" + str(self.__target_1_composition[0]) + "_" + str(self.__target_1_composition[1]) + "_" + str(self.__target_2_composition[0]) + "_" + str(self.__target_2_composition[1])
            if not os.path.exists(dir):
                os.makedirs(dir)
            dist = ((self.__current_position[0] - self.__target_position[0]) ** 2 +
                    (self.__current_position[1] - self.__target_position[1]) ** 2 +
                    (self.__current_position[2] - self.__target_position[2]) ** 2) ** 0.5
            ori_dist = ((self.__current_orientation[0] - self.__target_orientation[0]) ** 2 +
                    (self.__current_orientation[1] - self.__target_orientation[1]) ** 2) ** 0.5
            print dist, ori_dist
            if self.__counter < self.__counter_MAX and dist < 0.5 and ori_dist < 1. * math.pi / 180.:
                # and self.__cmd_vel_buffer is not None \
                # and 0 < abs(self.__cmd_vel_buffer.linear.x) < 0.01 \
                # and 0 < abs(self.__cmd_vel_buffer.linear.y) < 0.01 \
                # and 0 < abs(self.__cmd_vel_buffer.linear.z) < 0.01:
                cv2.imwrite(dir + "/sample-" + str(self.__counter) + ".jpg", masked_data)
                self.__counter += 1

        except CvBridgeError as e:
            print(e)
        # self.__image_buffer_lock.release()

    def set_orb_roll(self, roll):
        self.__orb_roll = roll

    def set_target_roll(self, target_roll):
        self.__target_roll = target_roll

    def set_target_compositions(self, p1, p2):
        self.__target_1_composition = p1
        self.__target_2_composition = p2

    def compute_control(self):
        error_roll = self.__target_roll - self.__orb_roll
        self.__control_goal_roll = error_roll

    def get_control_roll(self):
        return self.__control_goal_roll

    def set_target_position(self, position):
        self.__target_position = position

    def set_target_orientation(self, orientation):
        self.__target_orientation = orientation

    def set_current_position(self, position):
        self.__current_position = position

    def set_current_orientation(self, orientation):
        self.__current_orientation = orientation