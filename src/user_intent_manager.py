#!/usr/bin/env python

import roslib
# roslib.load_manifest('bebop_msgs')
import rospy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose

from tf import transformations
from constant import *

import math

from threading import *

class USER_INTENT_MANAGER(object):
    def __init__(self):
        # user intended position offset expressed by user control
        self.__intended_position_offset = None#[0,0,0]  # format: [delta_x, delta_y, delta_z]

        # only if there is no composed targets, __intended_orientation_offset and __intended_orientation are used
        self.__intended_orientation_offset = [0,0]  # format: [delta_pan_right, delta_tilt_down] in degree
        self.__intended_orientation = None     # format: [pan_world, tilt_world, roll_world] in radians

        # user intended composition for each target
        self.__intended_compositions = dict()  # format: dict {target.id : [u,v] or None}, where None means no intended composition for that target


        # take shot
        # rospy.Subscriber('/bebop/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=1)
        # self.__image_compressed_buffer = None
        # self.__image_compressed_buffer_lock = Lock()
        # self.__pub_snap_shot = rospy.Publisher('fleye/snap_shot', CompressedImage, queue_size=1)
        # self.__pub_snap_shot_info = rospy.Publisher('fleye/snap_shot_info', Float32MultiArray, queue_size=1)

    # def take_snap_shot(self, cam2world, compositions_of_interest):
    #     self.__image_compressed_buffer_lock.acquire()
    #     self.__pub_snap_shot.publish(self.__image_compressed_buffer)
    #     self.__image_compressed_buffer_lock.release()
    #
    #     pose_composition_msg = Float32MultiArray()
    #
    #     # pose
    #     pose_composition_msg.data.append(cam2world.position.x)
    #     pose_composition_msg.data.append(cam2world.position.y)
    #     pose_composition_msg.data.append(cam2world.position.z)
    #     pose_composition_msg.data.append(cam2world.orientation.x)
    #     pose_composition_msg.data.append(cam2world.orientation.y)
    #     pose_composition_msg.data.append(cam2world.orientation.z)
    #     pose_composition_msg.data.append(cam2world.orientation.w)
    #
    #     # composition
    #     for target_id in compositions_of_interest.keys():
    #         pose_composition_msg.data.append(target_id)
    #         if compositions_of_interest[target_id] is not None and \
    #             0 < compositions_of_interest[target_id][0] < IMAGE_WIDTH and \
    #             0 < compositions_of_interest[target_id][1] < IMAGE_HEIGHT:
    #             pose_composition_msg.data.append(compositions_of_interest[target_id][0])
    #             pose_composition_msg.data.append(compositions_of_interest[target_id][1])
    #
    #     self.__pub_snap_shot_info.publish(pose_composition_msg)

    # def image_compressed_callback(self, data):
    #     self.__image_compressed_buffer_lock.acquire()
    #     self.__image_compressed_buffer = data
    #     self.__image_compressed_buffer_lock.release()

    def cancel_all_targets(self):
        self.__intended_compositions.clear()

    def cancel_target(self, target_id):
        if self.__intended_compositions.has_key(target_id):
            del self.__intended_compositions[target_id]

    def set_target_composition(self, target_id, compositional_position):
        self.__intended_compositions[target_id] = compositional_position

    def get_composed_targets(self):
        composed_targets = []
        for target_id in self.__intended_compositions.keys():
            if self.__intended_compositions[target_id] is not None:
                composed_targets.append(target_id)
        return composed_targets

    def get_none_composed_targets(self):
        none_composed_targets = []
        for target_id in self.__intended_compositions.keys():
            if self.__intended_compositions[target_id] is None:
                none_composed_targets.append(target_id)
        return none_composed_targets

    def get_composed_compositions(self):
        composed_compositions = dict()
        for target_id in self.__intended_compositions:
            if self.__intended_compositions[target_id] is not None:
                composed_compositions[target_id] = self.__intended_compositions[target_id]
        return composed_compositions

    def get_compositions(self):
        return self.__intended_compositions

    # given a camera_pose and user control, compute the goal pose in FRAME_ID_WORLD
    def set_intention_from_user_control(self, cam2world_4x4, user_right, user_down, user_forward, user_pan_right, user_tilt_down):
        # intended position offset
        user_translation_camera_3x1 = np.array([[user_right, user_down, user_forward]]).T
        user_translation_world_3x1 = np.dot(cam2world_4x4[:3,:3], user_translation_camera_3x1)

        # user_translation_world_4x1 = np.dot(cam2world_4x4, user_translation_camera_4x1)
        self.__intended_position_offset = [user_translation_world_3x1[0,0], user_translation_world_3x1[1,0], user_translation_world_3x1[2,0]]
        self.__intended_orientation = None

        # intended compositions
        if len(self.get_composed_targets()) > 0:
            for target_id in self.get_composed_targets():
                self.__intended_compositions[target_id][0] += user_pan_right * GAIN_User_pan_px_per_unit
                self.__intended_compositions[target_id][1] += user_tilt_down * GAIN_User_tilt_px_per_unit
                print "USER: target", target_id, "change to", self.__intended_compositions[target_id][0], ",", self.__intended_compositions[target_id][1]
                if self.__intended_compositions[target_id][0] < 0 \
                        or self.__intended_compositions[target_id][0] > IMAGE_WIDTH \
                        or self.__intended_compositions[target_id][1] < 0 \
                        or self.__intended_compositions[target_id][1] > IMAGE_HEIGHT:
                    self.__intended_compositions[target_id] = None
                    print "USER: target", target_id, "out of view"

            # when there is some composed target, reset __intended_orientation_offset and __intended_orientation
            self.__intended_orientation_offset = [0,0]
        else:
            # when there is no composed targets, __intended_orientation_offset is used
            self.__intended_orientation_offset = [user_pan_right * GAIN_User_pan_radian_per_unit, user_tilt_down * GAIN_User_tilt_radian_per_unit]

    def reset_intention(self, rotation_world):
        self.__intended_position_offset = None#[0,0,0]
        self.__intended_orientation_offset = [0,0]
        self.__intended_orientation = transformations.euler_from_quaternion([rotation_world.x, rotation_world.y, rotation_world.z, rotation_world.w], axes='ryxz')

    def get_intended_position_offset(self):
        return self.__intended_position_offset

    def get_intended_orientation(self, rotation_world):
        if self.__intended_orientation is None:
            rotation_world_euler = transformations.euler_from_quaternion([rotation_world.x, rotation_world.y, rotation_world.z, rotation_world.w], 'ryxz')
            return [rotation_world_euler[0] - self.__intended_orientation_offset[0],   # pan right -> turn left -> decrease pan angle
                    rotation_world_euler[1] + self.__intended_orientation_offset[1],   # tilt down -> turn up -> increase tilt angle
                    rotation_world_euler[2]]
        return self.__intended_orientation
