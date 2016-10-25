#!/usr/bin/env python

import roslib
roslib.load_manifest('ORB_SLAM2')
import rospy
import tf
import numpy as np

from constant import *

from std_msgs.msg import Float32MultiArray, Float32, Header
from sensor_msgs.msg import PointCloud, ChannelFloat32, Image
from geometry_msgs.msg import Point, Point32, PointStamped, Transform, PoseStamped, Point, Quaternion

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

from ORB_SLAM2.msg import OrbSlamStamped

from tf import transformations

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag

class ORB_SLAM_MANAGER(object):
    def __init__(self):
        # rospy.Subscriber('/orb_slam/pose', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/orb_slam/tranformed_frame_pointcloud', PointCloud, self.transformed_frame_pointcloud_callback)

        rospy.Subscriber('orb_slam/orb_slam_info', OrbSlamStamped, self.__orb_slam_info_callback)

        self.__pub_keypoints = rospy.Publisher('fleye/keypoints', Float32MultiArray, queue_size=1)

        self.__pub_pcl_world = rospy.Publisher('fleye/pcl_world', PointCloud, queue_size=1)
        self.__pub_pcl_cam = rospy.Publisher('fleye/pcl_cam', PointCloud, queue_size=1)

        self.__header = None
        self.__cam2world = None
        self.__world2cam = None
        self.__pcl_world = None
        self.__pcl_cam = None

        # average distance between pcl to camera in camera frame
        self.__average_depth = None

        self.__last_header = None
        self.__last_cam2world = None

        self.__last_callback_time = None

        # Kalman fiter
        self.__position_velocity_filter = KalmanFilter (dim_x=6, dim_z=3)
        self.__position_velocity_filter.x = np.array([0., 0., 0., 0., 0., 0.])
        self.__position_velocity_filter.F = np.array([[1., 1. / ORB_SLAM_FREQUENCY, 0., 0., 0., 0.],
                                                      [0., 1., 0., 0., 0., 0.],
                                                      [0., 0., 1., 1./ORB_SLAM_FREQUENCY, 0., 0.],
                                                      [0., 0., 0., 1., 0., 0.],
                                                      [0., 0., 0., 0., 1., 1./ORB_SLAM_FREQUENCY],
                                                      [0., 0., 0., 0., 0., 1.]])
        self.__position_velocity_filter.H = np.array([[1., 0., 0., 0., 0., 0.],
                                                      [0., 0., 1., 0., 0., 0.],
                                                      [0., 0., 0., 0., 1., 0.]])
        self.__position_velocity_filter.P *= 10.
        self.__position_velocity_filter.R = np.eye(3) * ORB_R_std**2
        q = Q_discrete_white_noise(dim=2, dt=1. / ORB_SLAM_FREQUENCY, var=ORB_Q_std**2)
        self.__position_velocity_filter.Q = block_diag(q,q,q)

        # thread safe
        self.__lock = Lock()

        # debug
        self.__pub_x = rospy.Publisher('fleye/debug/x_world', Float32, queue_size=1)
        self.__pub_y = rospy.Publisher('fleye/debug/y_world', Float32, queue_size=1)
        self.__pub_z = rospy.Publisher('fleye/debug/z_world', Float32, queue_size=1)

        self.__pub_vx = rospy.Publisher('fleye/debug/vx_world', Float32, queue_size=1)
        self.__pub_vy = rospy.Publisher('fleye/debug/vy_world', Float32, queue_size=1)
        self.__pub_vz = rospy.Publisher('fleye/debug/vz_world', Float32, queue_size=1)

        self.__pub_filtered_x = rospy.Publisher('fleye/debug/filtered_x_world', Float32, queue_size=1)
        self.__pub_filtered_y = rospy.Publisher('fleye/debug/filtered_y_world', Float32, queue_size=1)
        self.__pub_filtered_z = rospy.Publisher('fleye/debug/filtered_z_world', Float32, queue_size=1)

        self.__pub_filtered_vx = rospy.Publisher('fleye/debug/filtered_vx_world', Float32, queue_size=1)
        self.__pub_filtered_vy = rospy.Publisher('fleye/debug/filtered_vy_world', Float32, queue_size=1)
        self.__pub_filtered_vz = rospy.Publisher('fleye/debug/filtered_vz_world', Float32, queue_size=1)

        self.__pub_orb_cam2world = rospy.Publisher('fleye/debug/orb_cam2world', PoseStamped, queue_size=1)

        # profiler
        # rospy.Subscriber('/bebop/image_raw', Image, self.__image_callback, queue_size=1)
        # rospy.Subscriber('/orb_slam/unscaled_pose', PoseStamped, self.__pose_callback, queue_size=1)
        # self.__x = []
        # self.__y = []
        # self.__z = []
        # self.__yaw = []
        # self.__pitch = []
        # self.__roll = []

    def __image_callback(self, data):
        self.__lock.acquire()
        print "ORB: image received", data.header.stamp.to_nsec(), "at", rospy.Time.now().to_nsec()
        self.__lock.release()

    def __pose_callback(self, data):
        self.__lock.acquire()
        print "ORB: pose info received", data.header.stamp.to_nsec(), "at", rospy.Time.now().to_nsec()
        self.__lock.release()

    def __orb_slam_info_callback(self, data):
        self.__lock.acquire()
        # print "ORB: slam info received", data.header.stamp.to_nsec(), "at", rospy.Time.now().to_nsec()
        self.__last_header = self.__header
        self.__last_cam2world = self.__cam2world

        # for Kalman filter
        # self.__x.append(data.cam2world.translation.x)
        # self.__y.append(data.cam2world.translation.x)
        # self.__z.append(data.cam2world.translation.x)
        # yaw, pitch, roll = transformations.euler_from_quaternion([data.cam2world.rotation.x, data.cam2world.rotation.y, data.cam2world.rotation.z, data.cam2world.rotation.w], axes='ryxz')
        # self.__yaw.append(yaw)
        # self.__pitch.append(pitch)
        # self.__roll.append(roll)

        # if len(self.__yaw) >= 30:
            # print "ORB: mean (stdev) x:", np.mean(self.__x), "(", np.std(self.__x), ") y:", np.mean(self.__y), "(", np.std(self.__y), ") z:", np.mean(self.__z), "(", np.std(self.__z), ")"
            # print "ORB: mean (stdev) yaw:", np.mean(self.__yaw), "(", np.std(self.__yaw), ") pitch:", np.mean(self.__pitch), "(", np.std(self.__pitch), ") roll:", np.mean(self.__roll), "(", np.std(self.__roll), ")"
        #     self.__x = []
        #     self.__y = []
        #     self.__z = []
        #     self.__yaw = []
        #     self.__pitch = []
        #     self.__roll = []


        self.__header = data.header
        self.__cam2world = data.cam2world
        self.__world2cam = data.world2cam
        self.__pcl_world = data.pcl_world
        self.__pcl_cam = data.pcl_cam

        keypoints_msg = Float32MultiArray()
        for i in range(len(self.__pcl_cam.points)):
            keypoints_msg.data.append(self.__pcl_cam.channels[0].values[i])
            keypoints_msg.data.append(self.__pcl_cam.channels[1].values[i])

        # if len(self.__pcl_cam.points) > 100:
        #     keypoints_xy = []
        #     keypoints_depth = []
        #     for i in range(len(self.__pcl_cam.points)):
        #         keypoints_xy.append((self.__pcl_cam.channels[0].values[i], self.__pcl_cam.channels[1].values[i]))
        #         keypoints_depth.append(self.__pcl_cam.channels[2].values[i])
        #     weights = map(lambda kp: ((IMAGE_WIDTH/2. - kp[0]) ** 2 + (IMAGE_HEIGHT/2. - kp[1])**2) **0.5, keypoints_xy)
        #     self.__average_depth = np.average(keypoints_depth, weights=weights)
        # else:
        #     self.__average_depth = None

        self.__pub_keypoints.publish(keypoints_msg)
        self.__pub_pcl_world.publish(self.__pcl_world)
        self.__pub_pcl_cam.publish(self.__pcl_cam)

        # update kalman filter
        # if self.__last_callback_time is not None and rospy.Time.now() - self.__last_callback_time > rospy.Duration(secs=0.1):
        #     old_state = self.__position_velocity_filter.x
        #     self.__position_velocity_filter.x = np.array([old_state[0], 0., old_state[2], 0., old_state[4], 0.])
        #     self.__position_velocity_filter.P *= 0.2
        # else:
        z = np.array([data.cam2world.translation.x, data.cam2world.translation.y, data.cam2world.translation.z])

        self.__position_velocity_filter.predict()
        self.__position_velocity_filter.update(z)



        # self.__last_callback_time = rospy.Time.now()

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

    def get_average_depth(self):
        return self.__average_depth

    def get_last_header_and_cam2world(self):
        self.__lock.acquire()
        header = self.__last_header
        cam2world = self.__last_cam2world
        self.__lock.release()
        return [header, cam2world]

    def get_position(self):
        self.__lock.acquire()
        position = [self.__position_velocity_filter.x[0] + self.__position_velocity_filter.x[1] * DELAY_predition_in_sec,
                    self.__position_velocity_filter.x[2] + self.__position_velocity_filter.x[3] * DELAY_predition_in_sec,
                    self.__position_velocity_filter.x[4] + self.__position_velocity_filter.x[5] * DELAY_predition_in_sec]
        self.__lock.release()
        return position

    def get_velocity(self):
        self.__lock.acquire()
        velocity = [self.__position_velocity_filter.x[1], self.__position_velocity_filter.x[3], self.__position_velocity_filter.x[5]]
        self.__lock.release()
        return velocity

    def get_orientation(self):
        self.__lock.acquire()
        yaw, pitch, roll = transformations.euler_from_quaternion([self.__cam2world.rotation.x, self.__cam2world.rotation.y, self.__cam2world.rotation.z, self.__cam2world.rotation.w], axes='ryxz')
        self.__lock.release()
        return [yaw, pitch, roll]

    def pub_debug_info(self):
        self.__lock.acquire()
        try:
            poseMsg = PoseStamped()
            poseMsg.header = self.__header
            poseMsg.header.frame_id = FRAME_ID_WORLD
            poseMsg.pose.position = Point(self.__cam2world.translation.x, self.__cam2world.translation.y, self.__cam2world.translation.z)
            poseMsg.pose.orientation = Quaternion(self.__cam2world.rotation.x, self.__cam2world.rotation.y, self.__cam2world.rotation.z, self.__cam2world.rotation.w)
            self.__pub_orb_cam2world.publish(poseMsg)

            self.__pub_filtered_x.publish(self.__position_velocity_filter.x[0])
            self.__pub_filtered_y.publish(self.__position_velocity_filter.x[2])
            self.__pub_filtered_z.publish(self.__position_velocity_filter.x[4])

            self.__pub_filtered_vx.publish(self.__position_velocity_filter.x[1])
            self.__pub_filtered_vy.publish(self.__position_velocity_filter.x[3])
            self.__pub_filtered_vz.publish(self.__position_velocity_filter.x[5])

            self.__pub_x.publish(self.__cam2world.translation.x)
            self.__pub_y.publish(self.__cam2world.translation.y)
            self.__pub_z.publish(self.__cam2world.translation.z)

            self.__pub_vx.publish((self.__cam2world.translation.x - self.__last_cam2world.translation.x) / (self.__header.stamp - self.__last_header.stamp).to_sec())
            self.__pub_vy.publish((self.__cam2world.translation.y - self.__last_cam2world.translation.y) / (self.__header.stamp - self.__last_header.stamp).to_sec())
            self.__pub_vz.publish((self.__cam2world.translation.z - self.__last_cam2world.translation.z) / (self.__header.stamp - self.__last_header.stamp).to_sec())
        except Exception, e:
            print "ORB: pub_debug_info failed: %s"%e
        finally:
            self.__lock.release()

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
