#!/usr/bin/env python
import roslib
# roslib.load_manifest('fleye_controller')
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Point, PointStamped, PoseStamped, Quaternion

import cmd_manager,angle
from tf import transformations
from constant import *
import math

import numpy as np

from fleye_controller.srv import ReflexxesControl, ReflexxesControlRequest

from std_msgs.msg import Float32

# SET -> COMPUTE -> GET
class CONTROLLER(object):
    def __init__(self):
        self.__current_image_position = None                # format: x, y, z
        self.__current_image_orientation = None             # format: pan_world, tilt_world, roll_world in radians
        self.__current_image_translation_velocity = None    # format: vx, vy, vz

        self.__current_compositions = dict()    # format: dict { target_id: (u, v) }
        self.__current_pan = None               # format: in radians
        self.__current_tilt = None              # format: in radians

        self.__target_image_position = None                 # format: x, y, z
        self.__target_image_translation_velocity = None     # format: vx, vy, vz
        self.__target_compositions = dict()                 # format: dict { target_id: (u, v) }, value = None is filtered out before passing in
        self.__target_pan_world = None              # useful only when __target_compositions is empty, in radians
        self.__target_tilt_world = None             # useful only when __target_compositions is empty, in radians

        self.__control_forward = None
        self.__control_left = None
        self.__control_up = None

        self.__control_turn_left = None
        self.__control_goal_pan = None              # in radians
        self.__control_goal_tilt = None             # in radians

        self.__control_turn_left_velocity = 0
        self.__control_pan_velocity = 0
        self.__control_tilt_velocity = 0
        self.__control_pan_acceleration = 0
        self.__control_tilt_acceleration = 0

        self.__srv_reflexxes_control = rospy.ServiceProxy('fleye/reflexxes_control', ReflexxesControl)

        # debug
        self.__pub_target_position = rospy.Publisher('fleye/debug/controller_target_position', PointStamped, queue_size=1)
        self.__pub_target_pose = rospy.Publisher('fleye/debug/controller_target_pose', PoseStamped, queue_size=1)

        self.__pub_virtualcamera_pan = rospy.Publisher('fleye/debug/virtual_camera_pan', Float32, queue_size=1)
        self.__pub_target_pan_world = rospy.Publisher('fleye/debug/target_pan_world', Float32, queue_size=1)
        self.__pub_current_pan_world = rospy.Publisher('fleye/debug/current_pan_world', Float32, queue_size=1)

        self.__pub_target_x_world = rospy.Publisher('fleye/debug/target_x_world', Float32, queue_size=1)
        self.__pub_target_y_world = rospy.Publisher('fleye/debug/target_y_world', Float32, queue_size=1)
        self.__pub_target_z_world = rospy.Publisher('fleye/debug/target_z_world', Float32, queue_size=1)

        self.__pub_current_x_world = rospy.Publisher('fleye/debug/current_x_world', Float32, queue_size=1)
        self.__pub_current_y_world = rospy.Publisher('fleye/debug/current_y_world', Float32, queue_size=1)
        self.__pub_current_z_world = rospy.Publisher('fleye/debug/current_z_world', Float32, queue_size=1)

        self.__pub_current_vx_world = rospy.Publisher('fleye/debug/current_vx_world', Float32, queue_size=1)
        self.__pub_current_vy_world = rospy.Publisher('fleye/debug/current_vy_world', Float32, queue_size=1)
        self.__pub_current_vz_world = rospy.Publisher('fleye/debug/current_vz_world', Float32, queue_size=1)

        self.__pub_reflexxes_x = rospy.Publisher('fleye/debug/reflexxes_x', Float32, queue_size=1)
        self.__pub_reflexxes_y = rospy.Publisher('fleye/debug/reflexxes_y', Float32, queue_size=1)
        self.__pub_reflexxes_z = rospy.Publisher('fleye/debug/reflexxes_z', Float32, queue_size=1)
        self.__pub_reflexxes_vx = rospy.Publisher('fleye/debug/reflexxes_vx', Float32, queue_size=1)
        self.__pub_reflexxes_vy = rospy.Publisher('fleye/debug/reflexxes_vy', Float32, queue_size=1)
        self.__pub_reflexxes_vz = rospy.Publisher('fleye/debug/reflexxes_vz', Float32, queue_size=1)
        self.__pub_reflexxes_ax = rospy.Publisher('fleye/debug/reflexxes_ax', Float32, queue_size=1)
        self.__pub_reflexxes_ay = rospy.Publisher('fleye/debug/reflexxes_ay', Float32, queue_size=1)
        self.__pub_reflexxes_az = rospy.Publisher('fleye/debug/reflexxes_az', Float32, queue_size=1)


    def set_current_state(self, image_position, image_orientation, image_translation_velocity, compositions, pan, tilt):
        self.__current_image_position = image_position
        self.__current_image_orientation = image_orientation # pan->tilt->roll
        self.__current_image_translation_velocity = image_translation_velocity

        self.__current_compositions = compositions
        self.__current_pan = pan
        self.__current_tilt = tilt

    def set_target_state(self, image_position, image_translation_velocity, compositions, rotation_world):
        self.__target_image_position = image_position
        self.__target_image_translation_velocity = image_translation_velocity
        self.__target_compositions = compositions
        self.__target_pan_world = rotation_world[0]
        self.__target_tilt_world = rotation_world[1]

    def compute_control(self):
        self.__pub_current_state_and_target_state()
        self.__compute_orientation_control()
        self.__compute_position_control()

    def get_control_forward(self):
        return self.__control_forward

    def get_control_left(self):
        return self.__control_left

    def get_control_up(self):
        return self.__control_up

    def get_control_turn_left(self):
        return self.__control_turn_left

    def get_control_goal_pan(self):
        return self.__control_goal_pan

    def get_control_goal_tilt(self):
        return self.__control_goal_tilt

    def __compute_orientation_control(self):
        if len(self.__target_compositions) > 0:
            try:
                errors_u = []
                errors_v = []
                for target_id in self.__target_compositions.keys():
                    if not self.__current_compositions.has_key(target_id):
                        print "CONTROLLER: inconsistent composition"
                        continue

                    if self.__current_compositions[target_id] is None:
                        print "CONTROLLER: target", target_id, "is behind the camera, this should be handled beforehand already"
                        continue

                    errors_u.append(self.__target_compositions[target_id][0] - self.__current_compositions[target_id][0])
                    errors_v.append(self.__target_compositions[target_id][1] - self.__current_compositions[target_id][1])

                error_u = np.average(errors_u)
                error_v = np.average(errors_v)

                if abs(error_u) < ERROR_TOLERANCE_Composition_pan:
                    error_u = 0
                if abs(error_v) < ERROR_TOLERANCE_Composition_tilt:
                    error_v = 0

                # image space control, no need reflexxes
                self.__control_goal_pan = self.__current_pan - error_u * GAIN_P_pan_radian_per_px
                self.__control_goal_tilt = self.__current_tilt + error_v * GAIN_P_tilt_radian_per_px

                # reflexxes is only used for __control_turn_left
                reflexxes_response = self.__srv_reflexxes_control(self.__get_reflexxes_orientation_control_request(0, 0))
                self.__control_turn_left = -reflexxes_response.vz #self.__control_goal_pan * GAIN_P_turn_per_radian if abs(self.__control_goal_pan) > ERROR_TOLERANCE_Control_yaw else 0

            except rospy.ServiceException, e:
                print "CONTROLLER: ReflexxesControl Service call failed: %s"%e

        else:
            try:
                error_pan = angle.rad_from_to(self.__current_image_orientation[0], self.__target_pan_world)
                error_tilt = angle.rad_from_to(self.__current_image_orientation[1], self.__target_tilt_world)
                if abs(error_pan) < ERROR_TOLERANCE_Control_pan:
                    error_pan = 0
                if abs(error_tilt) < ERROR_TOLERANCE_Control_tilt:
                    error_tilt = 0
                reflexxes_response = self.__srv_reflexxes_control(self.__get_reflexxes_orientation_control_request(self.__current_pan + error_pan, self.__current_tilt + error_tilt))

                # control = np.array([[reflexxes_response.x, reflexxes_response.y, reflexxes_response.vz]]).T
                self.__control_pan_velocity = reflexxes_response.vx
                self.__control_tilt_velocity = reflexxes_response.vy
                self.__control_pan_acceleration = reflexxes_response.ax
                self.__control_tilt_acceleration = reflexxes_response.ay

                self.__control_goal_pan = reflexxes_response.x
                self.__control_goal_tilt = reflexxes_response.y
                self.__control_turn_left = - reflexxes_response.vz

                # self.__pub_virtualcamera_pan.publish(self.__current_pan)
                # self.__pub_target_pan_world.publish(self.__target_pan_world)
                self.__pub_current_pan_world.publish(self.__current_image_orientation[0])
                # self.__pub_reflexxes_response(reflexxes_response)
                # self.__pub_pan_velocity_estimate.publish(self.__control_pan_velocity)

            except rospy.ServiceException, e:
                print "CONTROLLER: ReflexxesControl Service call failed: %s"%e

            ## p controller version [not fully tested], replaced by reflexxes
            # error_pan = angle.rad_from_to(self.__current_image_orientation[0], self.__target_pan_world)
            # error_tilt = angle.rad_from_to(self.__current_image_orientation[1], self.__target_tilt_world)
            #
            # self.__control_goal_pan = self.__current_pan + (error_pan * GAIN_P_pan_radian_per_radian if abs(error_pan) > ERROR_TOLERANCE_Control_pan else 0)
            # self.__control_goal_tilt = self.__current_tilt + (error_tilt * GAIN_P_tilt_radian_per_radian if abs(error_tilt) > ERROR_TOLERANCE_Control_tilt else 0)
            # self.__control_turn_left = - self.__control_goal_pan * GAIN_P_turn_per_radian if abs(self.__control_goal_pan) > ERROR_TOLERANCE_Control_yaw else 0
            # print "CONTROLLER: goal_pan -> control_turn_left", self.__control_goal_pan, self.__control_turn_left

    def __compute_position_control(self):
        # rospy.wait_for_service('fleye/reflexxes_control')
        try:
            reflexxes_response = self.__srv_reflexxes_control(self.__get_reflexxes_position_control_request())

            control_world = np.array([[reflexxes_response.vx, reflexxes_response.vy, reflexxes_response.vz]]).T

            # rotate control from world to drone, i.e., rotate around downward y-axis
            drone_yaw = self.__current_image_orientation[0] - self.__current_pan   # virtual camera offset
            # print "CONTROLLER: image_yaw", self.__current_image_orientation[0], "virtual camera pan", self.__current_pan, "drone_yaw", drone_yaw
            rotation_from_world_to_drone = transformations.rotation_matrix(-drone_yaw, [0,1,0])[:3,:3]
            control_drone = np.dot(rotation_from_world_to_drone, control_world)

            self.__control_left = - control_drone[0,0] * GAIN_R_left
            self.__control_up = - control_drone[1,0] * GAIN_R_up
            self.__control_forward = control_drone[2,0] * GAIN_R_forward

            self.__pub_reflexxes_response(reflexxes_response)

        except rospy.ServiceException, e:
            print "CONTROLLER: ReflexxesControl Service call failed: %s"%e

    def __get_reflexxes_orientation_control_request(self, goal_pan, goal_tilt):
        request = ReflexxesControlRequest()
        request.c_x = self.__current_pan    # pan
        request.c_y = self.__current_tilt   # tilt
        request.c_z = self.__current_pan if abs(self.__current_pan) > ERROR_TOLERANCE_Control_yaw else 0    # yaw

        request.c_vx = self.__control_pan_velocity
        request.c_vy = self.__control_tilt_velocity
        request.c_vz = 0 #self.__control_turn_left_velocity

        request.c_ax = self.__control_pan_acceleration
        request.c_ay = self.__control_tilt_acceleration
        request.c_az = 0 #

        request.b_vx = math.pi * 18./180.
        request.b_vy = math.pi * 18./180.
        request.b_vz = 1

        request.b_ax = 9999# math.pi# * 20./180.
        request.b_ay = 9999# math.pi# * 20./180.
        request.b_az = 2

        request.b_jx = 9999# * 20./180.
        request.b_jy = 9999# math.pi# * 20./180.
        request.b_jz = 5

        request.t_x = goal_pan
        request.t_y = goal_tilt
        request.t_z = 0

        request.t_vx = 0
        request.t_vy = 0
        request.t_vz = 0

        return request

    def __get_reflexxes_position_control_request(self):
        request = ReflexxesControlRequest()

        request.c_x = self.__current_image_position[0]
        request.c_y = self.__current_image_position[1]
        request.c_z = self.__current_image_position[2]

        request.c_vx = self.__current_image_translation_velocity[0]
        request.c_vy = self.__current_image_translation_velocity[1]
        request.c_vz = self.__current_image_translation_velocity[2]

        request.c_ax = 0
        request.c_ay = 0
        request.c_az = 0

        request.b_vx = 3
        request.b_vy = 3
        request.b_vz = 3

        request.b_ax = 9999
        request.b_ay = 9999
        request.b_az = 9999

        request.b_jx = 9999
        request.b_jy = 9999
        request.b_jz = 9999

        request.t_x = self.__target_image_position[0] if abs(self.__target_image_position[0] - self.__current_image_position[0]) > ERROR_TOLERANCE_Control_x else self.__current_image_position[0]
        request.t_y = self.__target_image_position[1] if abs(self.__target_image_position[1] - self.__current_image_position[1]) > ERROR_TOLERANCE_Control_y else self.__current_image_position[1]
        request.t_z = self.__target_image_position[2] if abs(self.__target_image_position[2] - self.__current_image_position[2]) > ERROR_TOLERANCE_Control_z else self.__current_image_position[2]

        request.t_vx = self.__target_image_translation_velocity[0]
        request.t_vy = self.__target_image_translation_velocity[1]
        request.t_vz = self.__target_image_translation_velocity[2]

        return request

    def __pub_current_state_and_target_state(self):
        self.__pub_current_x_world.publish(self.__current_image_position[0])
        self.__pub_current_y_world.publish(self.__current_image_position[1])
        self.__pub_current_z_world.publish(self.__current_image_position[2])

        self.__pub_target_x_world.publish(self.__target_image_position[0])
        self.__pub_target_y_world.publish(self.__target_image_position[1])
        self.__pub_target_z_world.publish(self.__target_image_position[2])

        self.__pub_current_vx_world.publish(self.__current_image_translation_velocity[0])
        self.__pub_current_vy_world.publish(self.__current_image_translation_velocity[1])
        self.__pub_current_vz_world.publish(self.__current_image_translation_velocity[2])


    def __pub_reflexxes_response(self, response):
        self.__pub_reflexxes_x.publish(response.x)
        self.__pub_reflexxes_y.publish(response.y)
        self.__pub_reflexxes_z.publish(response.z)
        self.__pub_reflexxes_vx.publish(response.vx)
        self.__pub_reflexxes_vy.publish(response.vy)
        self.__pub_reflexxes_vz.publish(response.vz)
        self.__pub_reflexxes_ax.publish(response.ax)
        self.__pub_reflexxes_ay.publish(response.ay)
        self.__pub_reflexxes_az.publish(response.az)

    def pub_debug_info(self):
        if self.__target_image_position is not None:
            target_position = PointStamped()
            target_position.header = Header()
            target_position.header.frame_id = FRAME_ID_WORLD
            target_position.point = Point(self.__target_image_position[0], self.__target_image_position[1], self.__target_image_position[2])

            self.__pub_target_position.publish(target_position)

            if self.__target_pan_world is not None and self.__target_tilt_world is not None:
                target_pose = PoseStamped()
                target_pose.header = Header()
                target_pose.header.frame_id = FRAME_ID_WORLD
                target_pose.pose.position = Point(self.__target_image_position[0], self.__target_image_position[1], self.__target_image_position[2])

                q = transformations.quaternion_from_euler(self.__target_pan_world - math.pi / 2, -self.__target_tilt_world, 0, axes='ryzx') # note the axis is 'ryzx'

                target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

                self.__pub_target_pose.publish(target_pose)


# # input:
# # goal_pose and current_pose are Pose in FRAME_ID_WORLD
# # v_x, v_y and v_z speed in FRAME_ID_WORLD
# # current_pan is virtual camera state in degree
# def compute_position_control(goal_pose, current_pose, v_x, v_y, v_z, current_pan):
#     # interpret Poses
#     goal_x, goal_y, goal_z, goal_rx, goal_ry, goal_rz = pose_to_xyzrpy(goal_pose)
#     current_x, current_y, current_z, current_rx, current_ry, current_rz = pose_to_xyzrpy(current_pose)
#
#     # virtual camera offset
#     current_ry += current_pan * math.pi / 180.  # current_ry in FRAME_ID_WORLD is rotation around downward y-axis, start from here current_ry represent drone's yaw
#     # current_rx += current_tilt * 180.0 / math.pi  # roll in FRAME_ID_WORLD is rotation around rightward x
#
#     # P and D errors, assume I errors are 0
#     p_error_in_world = np.array([[goal_x - current_x, goal_y - current_y, goal_z - current_z]]).T
#     d_error_in_world = np.array([[v_x, v_y, v_z]]).T
#
#     # rotate errors to drone's frame, i.e., rotation around downward y-axis
#     rotation_from_world_to_drone = transformations.rotation_matrix(current_ry, [0,1,0])[:3,:3]     # rotation around y-axis
#
#     p_error_in_drone = np.dot(rotation_from_world_to_drone, p_error_in_world)
#     d_error_in_drone = np.dot(rotation_from_world_to_drone, d_error_in_world)
#
#     # add up
#     left = -(p_error_in_drone[0][0] * GAIN_P_left - d_error_in_drone[0][0] * GAIN_D_left)
#     up = -(p_error_in_drone[1][0] * GAIN_P_up - d_error_in_drone[1][0] * GAIN_D_up)
#     forward = p_error_in_drone[3][0] * GAIN_P_forward - d_error_in_drone[3][0] * GAIN_D_forward
#
#     return forward, left, up
#
#
# def compute_orientation_control(goal_pose, current_pose):
#     # interpret Poses
#     goal_x, goal_y, goal_z, goal_rx, goal_ry, goal_rz = pose_to_xyzrpy(goal_pose)
#     current_x, current_y, current_z, current_rx, current_ry, current_rz = pose_to_xyzrpy(current_pose)
#
#     pan = - angle.rad_from_to(current_ry, goal_ry) * 180.0 / math.pi
#     tilt = - goal_rx * 180.0 / math.pi
#     turn_left = pan * GAIN_P_turn   # turn to make pan=0
#
#     return turn_left, pan, tilt
#
#
# def pose_to_xyzrpy(pose):
#     r_x, r_y, r_z = transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
#     return pose.position.x, pose.position.y, pose.position.z, r_x, r_y, r_z