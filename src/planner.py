#!/usr/bin/env python

import roslib

import rospy

from std_msgs.msg import Header, Empty, Float32, Float32MultiArray, String, Int32, Header
from sensor_msgs.msg import Image, CompressedImage, ChannelFloat32, PointCloud
from geometry_msgs.msg import Point, Quaternion, Point32, PointStamped, PoseStamped                                     #http://docs.ros.org/api/geometry_msgs/html/index-msg.html

from tf import transformations

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

from target import Target

from constant import *
from vector import *
from angle import *

class PLANNER(object):
    def __init__(self):
        # hover
        self.__hover_position = None    # format: [x_world, y_world, z_world]

        # restore
        self.__restore_position = None  # format: [x_world, y_world, z_world]
        self.__restore_orientation = None   #format: [pan_world, tilt_world, roll_world]
        self.__restore_compositions = None   #format: { target_id : (u,v) }

        # common for zigzag and circle
        self.__target = None            # format: Target
        self.__start_position = None    # format: [x_world, y_world, z_world]
        self.__distance_to_target = None# format: float
        self.__waypoints = []           # format: [[x_world, y_world, z_world]...]
        self.__waypoint_count = None

        # debug
        self.__pub_hover_x = rospy.Publisher('fleye/debug/hover_position_x', Float32, queue_size=1)
        self.__pub_hover_y = rospy.Publisher('fleye/debug/hover_position_y', Float32, queue_size=1)
        self.__pub_hover_z = rospy.Publisher('fleye/debug/hover_position_z', Float32, queue_size=1)


        self.__pub_hover_position = rospy.Publisher('fleye/debug/planner_hover_position', PointStamped, queue_size=1)
        self.__pub_restore_pose = rospy.Publisher('fleye/debug/planner_restore_pose', PoseStamped, queue_size=1)
        self.__pub_waypoints = rospy.Publisher('fleye/debug/planner_waypoints', PointCloud, queue_size=1)

    # --------------------------------------------- HOVER
    def set_hover_position(self, position):
        self.__hover_position = position

    def get_hover_position(self):
        return self.__hover_position

    def update_hover_position(self, position, offset_direction):
        hover_position_offset_1x3 = project_from_to(np.array([position[0] - self.__hover_position[0], position[1] - self.__hover_position[1], position[2] - self.__hover_position[2]]),
                                                    np.array([offset_direction[0], offset_direction[1], offset_direction[2]]))

        self.__hover_position[0] += 0 if math.isnan(hover_position_offset_1x3[0]) else hover_position_offset_1x3[0]
        self.__hover_position[1] += 0 if math.isnan(hover_position_offset_1x3[1]) else hover_position_offset_1x3[1]
        self.__hover_position[2] += 0 if math.isnan(hover_position_offset_1x3[2]) else hover_position_offset_1x3[2]


    # --------------------------------------------- RESTORE
    def plan_restore(self, position, orientation, compositions):
        self.__restore_position = position
        self.__restore_orientation = orientation
        self.__restore_compositions = compositions

    def is_restore_done(self, position, orientation, current_compositions):
        # check distance
        if distance_between(np.array(position), np.array(self.__restore_position)) > ERROR_TOLERANCE_Control_x:
            return False

        if len(self.__restore_compositions) is 0:
            if rad_from_to(orientation[0], self.__restore_orientation[0]) > ERROR_TOLERANCE_Control_pan * ERROR_TOLERANCE_Composition_pan_factor \
                or rad_from_to(orientation[1], self.__restore_orientation[1]) > ERROR_TOLERANCE_Control_tilt * ERROR_TOLERANCE_Composition_tilt_factor:
                return False
        else:
            for target_id in self.__restore_compositions.keys():
                if current_compositions[target_id] is None:
                    return False
                if not self.__is_target_well_composed(self.__restore_compositions[target_id], current_compositions[target_id]):
                    return False
        return True

    def reset_restore(self):
        self.__hover_position = self.__restore_position
        self.__restore_position = None
        self.__restore_orientation = None
        self.__restore_compositions = None

    def get_restore_position(self):
        return self.__restore_position

    def get_restore_orientation(self):
        return self.__restore_orientation

    def get_restore_compositions(self):
        return self.__restore_compositions

    # --------------------------------------------- ZIGZAG
    def plan_zigzag(self, target, number_of_rows=3, number_of_shots_each_side = 2, angle_interval = 20. * math.pi / 180.):  #TODO: number_of_rows
        self.__target = target
        self.__start_position = self.__hover_position               #TODO: self.__hover_position is not None
        self.__distance_to_target = distance_between(np.array(self.__target.get_center()), np.array(self.__start_position))

        self.__waypoints = []
        for i in range(2 * number_of_shots_each_side + 1):
            waypoint_1x3 = rotate(np.array(self.__start_position).T, angle_interval * (number_of_shots_each_side - i), (0,1,0), np.array(self.__target.get_center()).T)
            self.__waypoints.append([waypoint_1x3[0], waypoint_1x3[1], waypoint_1x3[2]])

        self.__waypoint_count = 0

    def is_zigzag_done(self):
        return self.__waypoint_count >= len(self.__waypoints)

    def reset_zigzag(self):
        self.__hover_position = self.__start_position
        self.__target = None
        self.__start_position = None
        self.__distance_to_target = None
        self.__waypoints = []
        self.__waypoint_count = None

    def is_zigzag_current_waypoint_reached(self, position, intended_composition, current_composition):
        return self.__is_zigzag_waypoint_reached(position, self.__waypoint_count, intended_composition, current_composition)

    def get_current_zigzag_waypoint(self):
        return self.__waypoints[self.__waypoint_count]

    def update_zigzag_waypoint(self):
        self.__waypoint_count += 1

    # TODO: implemement this to enable shared control
    # def update_position_during_zigzag(self, position, offset_direction):


    def __is_zigzag_waypoint_reached(self, position, waypoint_index, intended_composition, current_composition):

        # check composition
        if not self.__is_target_well_composed(intended_composition, current_composition):
            # print "PLANNER: zigzag\tbad composition"
            return False

        # check distance
        if abs(distance_between(np.array(self.__target.get_center()), np.array(position)) / self.__distance_to_target - 1) > ERROR_TOLERANCE_distance_ratio_to_target:
            # print "PLANNER: zigzag\tgood composition\tbad distance", abs(distance_between(np.array(self.__target.get_center()), np.array(position)) / self.__distance_to_target - 1)
            return False

        # check pan and tilt wrt target
        target_to_current = np.array(position) - np.array(self.__target.get_center())
        target_to_waypoint = np.array(self.__waypoints[waypoint_index]) - np.array(self.__target.get_center())

        if angle_between(target_to_current, target_to_waypoint) > ERROR_TOLERANCE_angle_difference_wrt_target:
            # print "PLANNER: zigzag\tgood composition\tgood distance\tbad angle", angle_between(target_to_current, target_to_waypoint) * 180. / math.pi
            return False

        return True



    # common for all
    def get_target(self):
        return self.__target

    def __is_target_well_composed(self, intended_composition, current_composition):
        return abs(intended_composition[0] - current_composition[0]) < ERROR_TOLERANCE_Composition_pan_factor * ERROR_TOLERANCE_Composition_pan \
            and abs(intended_composition[1] - current_composition[1]) < ERROR_TOLERANCE_Composition_tilt_factor * ERROR_TOLERANCE_Composition_tilt

    # --------------------------------------------- DEBUG
    def pub_debug_info(self):
        if self.__hover_position is not None:
            self.__pub_hover_x.publish(self.__hover_position[0])
            self.__pub_hover_y.publish(self.__hover_position[1])
            self.__pub_hover_z.publish(self.__hover_position[2])
            hoverPositionMsg = PointStamped()
            hoverPositionMsg.header = Header()
            hoverPositionMsg.header.frame_id = FRAME_ID_WORLD
            hoverPositionMsg.point = Point(self.__hover_position[0], self.__hover_position[1], self.__hover_position[2])

            self.__pub_hover_position.publish(hoverPositionMsg)

        if self.__restore_position is not None:
            restorePoseMsg = PoseStamped()
            restorePoseMsg.header = Header()
            restorePoseMsg.header.frame_id = FRAME_ID_WORLD
            restorePoseMsg.pose.position = Point(self.__restore_position[0], self.__restore_position[1], self.__restore_position[2])
            q = transformations.quaternion_from_euler(self.__restore_orientation[0], self.__restore_orientation[1], self.__restore_orientation[2], axes='ryxz')
            restorePoseMsg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            self.__pub_restore_pose.publish(restorePoseMsg)

        waypointsMsg = PointCloud()
        waypointsMsg.header = Header()
        waypointsMsg.header.frame_id = FRAME_ID_WORLD

        channel_current_waypoint = ChannelFloat32()
        channel_current_waypoint.name = "is current waypoint?"

        for i in range(len(self.__waypoints)):
            waypointsMsg.points.append(Point32(self.__waypoints[i][0], self.__waypoints[i][1], self.__waypoints[i][2]))
            channel_current_waypoint.values.append(1 if self.__waypoint_count is i else 0)

        waypointsMsg.channels.append(channel_current_waypoint)
        self.__pub_waypoints.publish(waypointsMsg)