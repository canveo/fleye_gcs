#!/usr/bin/env python

import roslib
roslib.load_manifest('ORB_SLAM2')
import rospy
# import tf
import numpy as np

from constant import *

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point, Point32, PointStamped, Transform

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

from ORB_SLAM2.msg import OrbSlamStamped

from tf import transformations

class ORB_SLAM_MANAGER(object):
    def __init__(self):
        # rospy.Subscriber('/orb_slam/pose', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/orb_slam/tranformed_frame_pointcloud', PointCloud, self.transformed_frame_pointcloud_callback)

        rospy.Subscriber('orb_slam/orb_slam_info', OrbSlamStamped, self.orb_slam_info_callback)

        self.__pub_keypoints = rospy.Publisher('fleye/keypoints', Float32MultiArray, queue_size=1)

        self.__pub_pcl_world = rospy.Publisher('fleye/pcl_world', PointCloud, queue_size=1)
        self.__pub_pcl_cam = rospy.Publisher('fleye/pcl_cam', PointCloud, queue_size=1)

        self.__header = None
        self.__cam2world = None
        self.__world2cam = None
        self.__pcl_world = None
        self.__pcl_cam = None

        self.__lock = Lock()

        # self.pub_targets = rospy.Publisher('fleye/targets', PointCloud, queue_size=5)
        #
        # # a collections of all target objects created in the history
        # self.targets = dict()   # key is target.id, value is target object
        # self.targets_lock = Lock()
        #
        # self.target_compositions = dict()   # key is target.id, value is (u,v, w, h)
        #
        # # pointcloud msg
        # self.transformed_frame_pointcloud = None
        # self.transformed_frame_pointcloud_lock = Lock()

    def orb_slam_info_callback(self, data):
        self.__lock.acquire()
        self.__header = data.header
        self.__cam2world = data.cam2world
        self.__world2cam = data.world2cam
        self.__pcl_world = data.pcl_world
        self.__pcl_cam = data.pcl_cam

        keypoints_msg = Float32MultiArray()
        for i in range(len(self.__pcl_cam.points)):
            keypoints_msg.data.append(self.__pcl_cam.channels[0].values[i])
            keypoints_msg.data.append(self.__pcl_cam.channels[1].values[i])

        self.__pub_keypoints.publish(keypoints_msg)
        self.__pub_pcl_world.publish(self.__pcl_world)
        self.__pub_pcl_cam.publish(self.__pcl_cam)
        self.__lock.release()

    def get_header(self):
        self.__lock.acquire()
        header = self.__header
        self.__lock.release()
        return header

    def get_cam2world(self):
        self.__lock.acquire()
        cam2world = self.__cam2world
        self.__lock.release()
        return cam2world

    def get_cam2world_as_matrix(self):
        self.__lock.acquire()
        cam2world = self.__cam2world
        self.__lock.release()

        translation = transformations.translation_matrix([cam2world.translation.x, cam2world.translation.y, cam2world.translation.z])
        rotation = transformations.quaternion_matrix([cam2world.rotation.x, cam2world.rotation.y, cam2world.rotation.z, cam2world.rotation.w])

        return np.dot(translation, rotation)

    def get_world2cam(self):
        self.__lock.acquire()
        world2cam = self.__world2cam
        self.__lock.release()
        return world2cam

    def get_world2cam_as_matrix(self):
        self.__lock.acquire()
        world2cam = self.__world2cam
        self.__lock.release()

        translation = transformations.translation_matrix([world2cam.translation.x, world2cam.translation.y, world2cam.translation.z])
        rotation = transformations.quaternion_matrix([world2cam.rotation.x, world2cam.rotation.y, world2cam.rotation.z, world2cam.rotation.w])

        return np.dot(translation, rotation)

    def get_pcl_world(self):
        self.__lock.acquire()
        pcl_world = self.__pcl_world
        self.__lock.release()
        return pcl_world

    def get_pcl_cam(self):
        self.__lock.acquire()
        pcl_cam = self.__pcl_cam
        self.__lock.release()
        return pcl_cam

    # def reset_all_targets(self):
    #     self.targets_lock.acquire()
    #     self.targets.clear()
    #     self.targets_lock.release()

    # def add_target(self, x, y, w, h):
    #     self.transformed_frame_pointcloud_lock.acquire()
    #     if self.transformed_frame_pointcloud is None:
    #         print "orb_slam_manager.py: add_target: Empty point cloud"
    #         return
    #
    #     target = Target(x, y, w, h, self.transformed_frame_pointcloud)
    #     self.transformed_frame_pointcloud_lock.release()
    #
    #     self.targets_lock.acquire()
    #     self.targets[target.id] = target
    #     self.targets_lock.release()

    # def transformed_frame_pointcloud_callback(self, data):
    #     self.transformed_frame_pointcloud_lock.acquire()
    #     self.transformed_frame_pointcloud = data
    #     self.transformed_frame_pointcloud_lock.release()
    #
    #     self.publish_targets(data.header)

    # def publish_targets(self, header):
    #     # targets in the form of PointCloud
    #     targets = PointCloud()
    #
    #     channel_radius = ChannelFloat32()
    #     channel_id = ChannelFloat32()
    #     channel_u = ChannelFloat32()
    #     channel_v = ChannelFloat32()
    #     channel_w = ChannelFloat32()
    #     channel_h = ChannelFloat32()
    #
    #     channel_radius.name = "radius"
    #     channel_id.name = "id"
    #     channel_u.name = "center u"
    #     channel_v.name = "center v"
    #     channel_w.name = "half width"
    #     channel_h.name = "half height"
    #
    #     targets.header = header
    #     targets.header.frame_id = FRAME_ID_WORLD
    #
    #     self.targets_lock.acquire()
    #     num_of_targets = len(self.targets)
    #     for t in self.targets.items():
    #         targets.points.append(Point32(t.x, t.y, t.z))
    #         channel_radius.values.append(t.radius)
    #         channel_id.values.append(t.id)
    #         channel_u.values.append(0)
    #         channel_v.values.append(0)
    #         channel_w.values.append(0)
    #         channel_h.values.append(0)
    #     self.targets_lock.release()
    #
    #     # convert targets to camera coordinate
    #     targets_in_cam = self.tl.transformPointCloud(FRAME_ID_CAMERA, targets)
    #
    #     # fill u, v, w, h channels
    #     for i in range(num_of_targets):
    #         t_on_screen_3x1 = np.dot(CAMERA_MATRIX, np.array([[targets_in_cam.points[i].x, targets_in_cam.points[i].y, targets_in_cam.points[i].z]]).T)
    #         wh_on_screen_3x1 = np.dot(CAMERA_MATRIX, np.array([[channel_radius.values[i], channel_radius.values[i], targets_in_cam.points[i].z]]).T)
    #
    #         channel_u.values[i] = t_on_screen_3x1[0,0] / t_on_screen_3x1[0,2]
    #         channel_v.values[i] = t_on_screen_3x1[0,1] / t_on_screen_3x1[0,2]
    #
    #         channel_w.values[i] = wh_on_screen_3x1[0,0] / wh_on_screen_3x1[0,2]
    #         channel_h.values[i] = wh_on_screen_3x1[0,1] / wh_on_screen_3x1[0,2]
    #
    #         self.target_compositions[channel_id.values[i]] = [channel_u.values[i], channel_v.values[i], channel_w.values[i], channel_h.values[i]]
    #
    #     targets.channels.append(channel_radius)
    #     targets.channels.append(channel_id)
    #     targets.channels.append(channel_u)
    #     targets.channels.append(channel_v)
    #     targets.channels.append(channel_w)
    #     targets.channels.append(channel_h)
    #
    #     self.pub_targets.publish(targets)
