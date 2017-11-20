#!/usr/bin/env python

import roslib

import rospy

import numpy as np

# import cv2
# from cv_bridge import CvBridge, CvBridgeError

# from std_msgs.msg import Header, Empty, Float32, Float32MultiArray, String, Int32, Bool
# from sensor_msgs.msg import Image, CompressedImage, ChannelFloat32, PointCloud, Joy
# from geometry_msgs.msg import Pose, PoseStamped, Point32, Twist                                     #http://docs.ros.org/api/geometry_msgs/html/index-msg.html

# from nav_msgs.msg import Path

from tf import transformations

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
# from threading import Lock

# import cmd_manager, imu_manager, angle, orb_slam_manager, user_intent_manager, target, controller, planner
# from target import Target

# from constant import *
# from vector import *

def angle_between(v1, v2):
    v_dot = np.dot(v1.T,v2)
    # print 'v_dot', v_dot
    v_cross = np.cross(v1.T, v2.T).T
    # print 'v_cross', v_cross
    cos_angle = v_dot / (np.linalg.norm(v1) * np.linalg.norm(v2))
    sin_angle = np.linalg.norm(v_cross) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return cos_angle, sin_angle

def rotation(tcw, q1w, q2w, p1hat, p2hat):
    len_CP1 = np.linalg.norm(p1hat)
    len_CP2 = np.linalg.norm(p2hat)
    v_CQ1 = q1w - tcw
    v_CQ2 = q2w - tcw
    v_CP1 = len_CP1 * v_CQ1 / np.linalg.norm(v_CQ1)
    v_CP2 = len_CP2 * v_CQ2 / np.linalg.norm(v_CQ2)
    p1w = tcw + v_CP1
    p2w = tcw + v_CP2

    M_c = np.concatenate((np.zeros((3,1)).T, p1hat.T, p2hat.T)).T
    # print 'M_c', M_c
    M_w = np.concatenate((tcw.T, p1w.T, p2w.T)).T
    # print 'M_w', M_w

    mean_c = np.mean(M_c, axis=1).reshape((-1,1))
    # print 'mean_c', mean_c
    mean_w = np.mean(M_w, axis=1).reshape((-1,1))
    # print 'mean_w', mean_w

    M_c_norm = M_c - np.dot(mean_c, np.ones((1,3)))
    M_w_norm = M_w - np.dot(mean_w, np.ones((1,3)))

    M = np.dot(M_w_norm, M_c_norm.T)

    U, s, V = np.linalg.svd(M, full_matrices=True)
    # print 'U', U
    # print 'V', V
    # print 'check svd', np.dot(np.dot(U, np.diag(s)), V) - M


    s_proper = np.diag(np.array([1,1,np.linalg.det(np.dot(U, V))]))
    return np.dot(np.dot(U, s_proper), V)


def two_compose(t0w, q1w, q2w, p1i, p2i, epsilon1, epsilon2):
    # print "inputs", t0w, q1w, q2w, p1i, p2i, epsilon1, epsilon2

    K = np.array([[378.263308, 0, 321.342736],
                  [0, 373.648186, 183.469983],
                  [0,0,1]])
    K_inv = np.linalg.inv(K)
    p1hat = np.dot(K_inv, p1i)
    p2hat = np.dot(K_inv, p2i)

    # print 'p1hat', p1hat
    # print 'p2hat', p2hat

    cos_omega, sin_omega = angle_between(p1hat, p2hat)

    # print 'cos_omega', cos_omega
    # print 'sin_omega', sin_omega

    # basic vectors
    qmidw = (q1w + q2w) / 2.0
    # print 'qmidw', qmidw
    v_Q1Q2 = q2w - q1w
    v_T0Q1 = q1w - t0w
    v_T0Q2 = q2w - t0w

    # basic length
    len_Q1Qmid = np.linalg.norm(v_Q1Q2) / 2.0
    len_radius = len_Q1Qmid / sin_omega
    len_QmidX = len_radius * cos_omega
    # print "check len_QmidX", (len_radius**2 - len_Q1Qmid**2)**0.5 - len_QmidX

    # compute the center of the circle
    v_perp = np.cross(-v_Q1Q2.T, -v_T0Q1.T).T
    # print 'v_perp', v_perp
    # print 'check v_perp', np.dot(v_perp.T, v_T0Q1), np.dot(v_perp.T, v_T0Q2), np.dot(v_perp.T, v_Q1Q2)
    dir_QmidX = np.cross(v_Q1Q2.T, v_perp.T).T
    # print 'check dir_QmidX', np.dot(dir_QmidX.T, v_Q1Q2), np.linalg.norm(qmidw + dir_QmidX - q1w) - np.linalg.norm(qmidw + dir_QmidX - q2w)
    v_QmidX = len_QmidX * dir_QmidX / np.linalg.norm(dir_QmidX)
    # print 'check v_QmidX', np.dot(dir_QmidX.T, v_QmidX) / (np.linalg.norm(dir_QmidX) * np.linalg.norm(v_QmidX)) - 1
    xw = qmidw + v_QmidX
    # print 'xw', xw
    # print 'check xw', np.linalg.norm(xw - q1w) - len_radius, np.linalg.norm(xw - q2w) - len_radius

    # compute the (possible) best camera c0w
    v_XT0 = t0w - xw
    c0w = xw + len_radius * v_XT0 / np.linalg.norm(v_XT0)
    # print 'c0w', c0w

    v_C0Q1 = q1w - c0w
    v_C0Q2 = q2w - c0w
    cos_omega_check, sin_omega_check = angle_between(v_C0Q1, v_C0Q2)
    # print 'cos_omega_check', cos_omega_check
    # print 'sin_omega_check', sin_omega_check

    if np.linalg.norm(v_C0Q1) >= epsilon1 and \
        np.linalg.norm(v_C0Q2) >= epsilon2 and \
        abs(cos_omega - cos_omega_check) < 0.001 and \
        abs(sin_omega - sin_omega_check) < 0.001:
        r0w = rotation(c0w,q1w,q2w,p1hat,p2hat)
        p1raw = np.dot(r0w.T, (q1w-c0w))
        p2raw = np.dot(r0w.T, (q2w-c0w))
        print "should be zero", p1raw/p1raw[2] - p1hat,  p2raw/p2raw[2] - p2hat
        return c0w, rotation(c0w,q1w,q2w,p1hat,p2hat)

    # compute the boundary points c1w, c2w
    v_XQ1 = q1w - xw
    v_XQ2 = q2w - xw

    sin_theta1 = epsilon1 / 2.0 / len_radius
    sin_theta2 = epsilon2 / 2.0 / len_radius

    mat_R1 = transformations.rotation_matrix( 2 * np.arcsin(sin_theta1), v_perp)[:3,:3]
    mat_R2 = transformations.rotation_matrix(-2 * np.arcsin(sin_theta2), v_perp)[:3,:3]

    # print 'mat_R1', mat_R1
    # print 'mat_R2', mat_R2

    c1w = xw + np.dot(mat_R1, v_XQ1)
    c2w = xw + np.dot(mat_R2, v_XQ2)

    # print "check e1", np.linalg.norm(c1w - q1w) - epsilon1
    # print "check e2", np.linalg.norm(c2w - q2w) - epsilon2

    # print "check len_XQ1", np.linalg.norm(v_XQ1) - len_radius
    # print "check len_XQ2", np.linalg.norm(v_XQ2) - len_radius

    if np.linalg.norm(c1w - t0w) <= np.linalg.norm(c2w - t0w):
        r1w = rotation(c1w,q1w,q2w,p1hat,p2hat)
        p1raw = np.dot(r1w.T, (q1w-c1w))
        p2raw = np.dot(r1w.T, (q2w-c1w))
        print "should be zero", p1raw/p1raw[2] - p1hat,  p2raw/p2raw[2] - p2hat
        return c1w, r1w
    else:
        r2w = rotation(c2w,q1w,q2w,p1hat,p2hat)
        p1raw = np.dot(r2w.T, (q1w-c2w))
        p2raw = np.dot(r2w.T, (q2w-c2w))
        print "should be zero", p1raw/p1raw[2] - p1hat,  p2raw/p2raw[2] - p2hat
        return c2w, r2w

def rotation_to_minize_reprojection_error(tcw, q1w, q2w, p1i, p2i):
    # print "inputs", t0w, q1w, q2w, p1i, p2i, epsilon1, epsilon2

    K = np.array([[378.263308, 0, 321.342736],
                  [0, 373.648186, 183.469983],
                  [0,0,1]])
    K_inv = np.linalg.inv(K)
    p1hat = np.dot(K_inv, p1i)
    p2hat = np.dot(K_inv, p2i)
    cos_P1CP2, sin_P1CP2 = angle_between(p1hat, p2hat)

    v_CQ1 = q1w - tcw
    v_CQ2 = q2w - tcw
    v_Q1Q2 = q2w - q1w
    # cos_Q1CQ2, sin_Q1CQ2 = angle_between(v_CQ1, v_CQ2)

    d1 = np.linalg.norm(p1hat)
    d2 = np.linalg.norm(p2hat)

    if d1 > d2:
        # d_long = d1
        # d_short = d2
        v_perp = np.cross(v_CQ1.T, v_Q1Q2.T).T
        mat_rot = transformations.rotation_matrix(np.arcsin(sin_P1CP2), v_perp)[:3,:3]
        q2w_new = tcw + np.dot(mat_rot, v_CQ1)
        # print 'd1 is longer'

        # v_CP1 = v_CQ1 / np.linalg.norm(v_CQ1) * np.linalg.norm(p1hat)
        # p1w = tcw + v_CP1
        #
        # v_CP2 = v_CQ2_new / np.linalg.norm(v_CQ2_new) * np.linalg.norm(p2hat)
        # p2w = tcw + v_CP2

        return rotation(tcw, q1w, q2w_new, p1hat, p2hat)

    else:
        # d_long = d2
        # d_short = d1
        v_perp = np.cross(v_CQ1.T, -v_Q1Q2.T).T
        # print 'v_perp', v_perp
        mat_rot = transformations.rotation_matrix(np.arcsin(sin_P1CP2), v_perp)[:3,:3]
        q1w_new = tcw + np.dot(mat_rot, v_CQ2)
        # print 'd2 is longer'

        # v_CP2 = v_CQ2 / np.linalg.norm(v_CQ2) * np.linalg.norm(p2hat)
        # p2w = tcw + v_CP2
        #
        # v_CP1 = v_CQ1_new / np.linalg.norm(v_CQ1_new) * np.linalg.norm(p1hat)
        # p1w = tcw + v_CP1

        return rotation(tcw, q1w_new, q2w, p1hat, p2hat)
