#!/usr/bin/env python

import rospy

from sensor_msgs.msg import PointCloud, ChannelFloat32
from geometry_msgs.msg import Point, Point32

import numpy as np

# import tf
from tf import transformations

from constant import *
from vector import *

from threading import Lock

class Target(object):
    # x, y is the upper-left corner of a bb
    # w, h is the width and height of a bb
    # cam2world_4x4 is the cam2world transformation matrix
    # pcl_cam is the PointCloud in the current frame expressed in FRAME_ID_CAMERA
    def __init__(self, x, y, w, h, header, world2cam_4x4, pcl_world):
        # set id
        self.__id = header.seq

        selected_mappoints_cam = []
        selected_keypoints = []
        for i in range(len(pcl_world.points)):
            if x < pcl_world.channels[0].values[i] < x+w and y < pcl_world.channels[1].values[i] < y+h:
                selected_mappoints_cam.append(pcl_world.points[i])
                selected_keypoints.append((pcl_world.channels[0].values[i], pcl_world.channels[1].values[i]))

        if len(selected_keypoints) < 5:
            self.__is_valid = False
            return
        self.__is_valid = True

        # weight depend on distance between the keypoint and the bb center
        weights = map(lambda kp: ((x+w/2. - kp[0]) ** 2 + (y+h/2. - kp[1])**2) **0.5, selected_keypoints)
        # print "weights are ", weights

        selected_xs_world = map(lambda pt: pt.x, selected_mappoints_cam)
        selected_ys_world = map(lambda pt: pt.y, selected_mappoints_cam)
        selected_zs_world = map(lambda pt: pt.z, selected_mappoints_cam)
        self.__x = np.average(selected_xs_world, weights=weights)
        self.__y = np.average(selected_ys_world, weights=weights)
        self.__z = np.average(selected_zs_world, weights=weights)

        center_world_4x1 = np.array([[self.__x, self.__y, self.__z, 1]]).T

        # # x,y,z in FRAME_ID_CAMERA
        center_cam_4x1 = np.dot(world2cam_4x4, center_world_4x1)

        # radius
        wh_cam_3x1 = np.dot(CAMERA_MATRIX_INV, np.array([[w/2. + CAMERA_Cx, h/2. + CAMERA_Cy, 1]]).T)
        wh_cam_3x1_homo = wh_cam_3x1 / wh_cam_3x1[2,0]
        self.__radius = center_cam_4x1[2,0] * (wh_cam_3x1_homo[0,0] + wh_cam_3x1_homo[1,0]) / 2.

    def get_id(self):
        return self.__id

    def get_center(self):
        return self.__x, self.__y, self.__z

    def get_center_as_point32(self):
        return Point32(self.__x, self.__y, self.__z)

    def get_center_3x1(self):
        return np.array([[self.__x, self.__y, self.__z]]).T

    def get_center_4x1(self):
        return np.array([[self.__x, self.__y, self.__z, 1]]).T

    def get_radius(self):
        return self.__radius

    def is_valid(self):
        return self.__is_valid

class TARGET_MANAGER(object):
    def __init__(self):
        # all targets, including cancelled ones
        self.__targets = dict() # id->target object

        self.__compositions = dict()    # id->[u, v, half_w, half_h] if target is not behind screen, otherwise None

        self.__lock = Lock()

        # publish targets info
        self.__pub = rospy.Publisher('fleye/targets', PointCloud, queue_size=1)

    def add_target(self, x, y, w, h, header, world2cam_4x4, pcl_world):
        target = Target(x, y, w, h, header, world2cam_4x4, pcl_world)

        # could be an invalid target (with very few features)
        if not target.is_valid():
            return target

        self.__targets[target.get_id()] = target
        self.__compositions[target.get_id()] = self.__compute_target_composition(target.get_id(), world2cam_4x4)
        return target

    def get_target(self, target_id):
        return self.__targets[target_id]

    def get_target_composition(self, target_id):
        self.__lock.acquire()
        composition = self.__compositions[target_id]
        self.__lock.release()
        return composition

    def get_target_compositions(self, target_ids):
        self.__lock.acquire()
        compositions = dict()
        for target_id in target_ids:
            compositions[target_id] = self.__compositions[target_id]
        self.__lock.release()
        return compositions

    # input: a list of target id
    # output: PointCloud message
    def update_compositions_and_publish_targets_as_pcl(self, header, world2cam_4x4, compositions):#compositions include both composed_targets and none_composed_targets

        self.__update_compositions(world2cam_4x4)

        # targets in the form of PointCloud
        targetsMsg = PointCloud()

        channel_radius = ChannelFloat32()
        channel_id = ChannelFloat32()
        channel_u = ChannelFloat32()
        channel_v = ChannelFloat32()
        channel_w = ChannelFloat32()
        channel_h = ChannelFloat32()
        channel_c = ChannelFloat32()
        channel_tu = ChannelFloat32()
        channel_tv = ChannelFloat32()

        channel_radius.name = "radius"
        channel_id.name = "id"
        channel_u.name = "center u"
        channel_v.name = "center v"
        channel_w.name = "half width"
        channel_h.name = "half height"

        channel_c.name = "1 for composed, -1 for none_composed"
        channel_tu.name = "target u"
        channel_tv.name = "target v"

        targetsMsg.header = header
        targetsMsg.header.frame_id = FRAME_ID_WORLD

        for target_id in compositions.keys():
            if self.__compositions[target_id] is not None:
                targetsMsg.points.append(self.__targets[target_id].get_center_as_point32())
                channel_radius.values.append(self.__targets[target_id].get_radius())
                channel_id.values.append(target_id)
                channel_u.values.append(self.__compositions[target_id][0])
                channel_v.values.append(self.__compositions[target_id][1])
                channel_w.values.append(self.__compositions[target_id][2])
                channel_h.values.append(self.__compositions[target_id][3])
                if compositions[target_id] is not None:
                    channel_c.values.append(1.0)
                    channel_tu.values.append(compositions[target_id][0])
                    channel_tv.values.append(compositions[target_id][1])
                else:
                    channel_c.values.append(-1.0)
                    channel_tu.values.append(self.__compositions[target_id][0])
                    channel_tv.values.append(self.__compositions[target_id][1])

        targetsMsg.channels.append(channel_radius)
        targetsMsg.channels.append(channel_id)
        targetsMsg.channels.append(channel_u)
        targetsMsg.channels.append(channel_v)
        targetsMsg.channels.append(channel_w)
        targetsMsg.channels.append(channel_h)
        targetsMsg.channels.append(channel_c)
        targetsMsg.channels.append(channel_tu)
        targetsMsg.channels.append(channel_tv)

        self.__pub.publish(targetsMsg)

    def __update_compositions(self, world2cam_4x4):
        self.__lock.acquire()

        self.__compositions.clear()

        for target_id in self.__targets.keys():
            self.__compositions[target_id] = self.__compute_target_composition(target_id, world2cam_4x4)

        self.__lock.release()

    def __compute_target_composition(self, target_id, world2cam_4x4):
        target_cam_4x1 = np.dot(world2cam_4x4, self.__targets[target_id].get_center_4x1())

        # behind screen
        if target_cam_4x1[2,0] < 0:
            return None

        # target on screen
        composition_3x1 = np.dot(CAMERA_MATRIX, target_cam_4x1[:3])
        u = composition_3x1[0,0] / composition_3x1[2,0]
        v = composition_3x1[1,0] / composition_3x1[2,0]

        # half of width and half of height of the bb on screen
        wh_cam_4x1 = np.array([[self.__targets[target_id].get_radius(), self.__targets[target_id].get_radius(), target_cam_4x1[2,0], 1]]).T
        wh_screen_3x1 = np.dot(CAMERA_MATRIX, wh_cam_4x1[:3])
        half_w = wh_screen_3x1[0,0] / wh_screen_3x1[2,0] - CAMERA_Cx
        half_h = wh_screen_3x1[1,0] / wh_screen_3x1[2,0] - CAMERA_Cy
        return [u, v, half_w, half_h]

    # position: format is [x_world, y_world, z_world]
    # composition: format is [u,v]
    # return: [pan_world, tilt_world]
    def compute_camera_orientation(self, position, target_id, composition, is_testing = False):
        # R_c2w * C^-1 * [u,v,1].T = Target_world - t_c2w
        # lhs = C^-1 * [u,v,1].T AND rhs = Target_world - t_c2w
        lhs_3x1 = np.dot(CAMERA_MATRIX_INV, np.array([[composition[0], composition[1], 1]]).T)
        rhs_3x1 = self.__targets[target_id].get_center_3x1() - np.array([position]).T

        lhs_3x1_u = lhs_3x1 / length_of_vector(lhs_3x1)
        rhs_3x1_u = rhs_3x1 / length_of_vector(rhs_3x1)

        X_l, Y_l, Z_l = lhs_3x1_u[0,0], lhs_3x1_u[1,0], lhs_3x1_u[2,0]
        X_r, Y_r, Z_r = rhs_3x1_u[0,0], rhs_3x1_u[1,0], rhs_3x1_u[2,0]

        bb_4ac = Z_l**2 + Y_l**2 - Y_r**2
        if bb_4ac < 0:
            return None # sqrt of b^2 - 4ac is bad

        cos_t1 = (Y_l * Y_r + Z_l * bb_4ac ** 0.5) / (Z_l**2 + Y_l**2)
        cos_t2 = (Y_l * Y_r - Z_l * bb_4ac ** 0.5) / (Z_l**2 + Y_l**2)

        cos_t = cos_t1 if cos_t1 > cos_t2 else cos_t2

        if cos_t < 0:
            return None # tilt angle in Quadrant II or III

        sin_t = (cos_t * Y_l - Y_r) / Z_l

        tilt = np.arccos(cos_t) * (1 if sin_t > 0 else -1)

        b = sin_t * Y_l + cos_t * Z_l
        sin_p = (X_r * b - Z_r * X_l) / (b**2 + X_l**2)
        cos_p = (Z_r * b + X_r * X_l) / (b**2 + X_l**2)

        pan = np.arccos(cos_p) * (1 if sin_p > 0 else -1)

        return [pan, tilt]

    # position: format is [x_world, y_world, z_world]
    # orientation: format is [pan_world, tilt_world]
    def compute_composition_error(self, position, orientation, target_id, composition):
        R_4x4 = transformations.euler_matrix(orientation[0], orientation[1], 0, 'ryxz')
        T_4x4 = transformations.translation_matrix(position)

        world2cam_4x4 = inv(transformations.concatenate_matrices(T_4x4, R_4x4))

        current_composition = self.__compute_target_composition(target_id, world2cam_4x4)

        if current_composition is None:
            return 9999
        return ((composition[0] - current_composition[0])**2 + (composition[1] - current_composition[1])**2)**0.5