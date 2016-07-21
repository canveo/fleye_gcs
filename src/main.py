#!/usr/bin/env python

import roslib

import rospy

from std_msgs.msg import Header, Empty, Float32, Float32MultiArray, String, Int32, Bool
from sensor_msgs.msg import Image, CompressedImage, ChannelFloat32, PointCloud
from geometry_msgs.msg import Pose, PoseStamped, Point32                                     #http://docs.ros.org/api/geometry_msgs/html/index-msg.html

from tf import transformations

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

import cmd_manager, imu_manager, angle, orb_slam_manager, user_intent_manager, target, controller, planner
from target import Target

from constant import *
from vector import *

import Queue

class STAGE():
    stage_init      =0
    stage_idle      =1
    stage_orbit    =2
    stage_zigzag    =3
    stage_pano      =4
    stage_restore   =5

class BEBOP_GCS(object):
    def __init__(self):
        rospy.init_node('fleye_gcs', anonymous=True)

        rospy.Subscriber('fleye/gesture', Float32MultiArray, self.gesture_callback)
        self.pub_tl = rospy.Publisher('fleye/takeoff_land', String, queue_size=1)
        self.pub_pano = rospy.Publisher('fleye/pano', String, queue_size=1)
        self.pub_orbit = rospy.Publisher('fleye/orbit', String, queue_size=1)
        self.pub_zigzag = rospy.Publisher('fleye/zigzag', String, queue_size=1)

        # take shot
        rospy.Subscriber('/bebop/image_raw/compressed', CompressedImage, self.image_compressed_callback, queue_size=1)
        self.__pub_snap_shot_on_board = rospy.Publisher('bebop/snapshot', Empty, queue_size=1)
        self.__image_compressed_buffer = None
        self.__image_compressed_buffer_lock = Lock()
        self.__pub_snap_shot = rospy.Publisher('fleye/snap_shot', CompressedImage, queue_size=1)
        self.__pub_snap_shot_info = rospy.Publisher('fleye/snap_shot_info', Float32MultiArray, queue_size=1)

        # modules
        self.__cmd = cmd_manager.CMD_MANAGER()
        self.__imu = imu_manager.IMU_MANAGER()
        self.__orb = orb_slam_manager.ORB_SLAM_MANAGER()
        self.__user = user_intent_manager.USER_INTENT_MANAGER()
        self.__target_manager = target.TARGET_MANAGER()
        self.__controller = controller.CONTROLLER()
        self.__planner = planner.PLANNER()
        # self.tld = tld_module.TLD_MODULE()

        # stage
        self.__stage = STAGE.stage_idle

        # to check connection lost
        self.__last_header = None
        self.__is_last_header_new = False

        # debug
        rospy.Subscriber('fleye/debug/keyboard_overtake', Bool, self.overtake_callback)
        self.__is_overtaken = False

        self.__pub_hover_positions = rospy.Publisher('fleye/debug/main_hover_positions', PointCloud, queue_size=1)

        # MAIN LOOP
        rospy.Timer(rospy.Duration(1./GCS_LOOP_FREQUENCY), self.safe_main_routine)

    def overtake_callback(self, data):
        self.__is_overtaken = data.data

    def pub_interface_look(self):
        if self.__stage is STAGE.stage_idle:
            self.pub_pano.publish("white")
            self.pub_orbit.publish("white" if len(self.__user.get_composed_targets()) is 1 else "grey")
            self.pub_zigzag.publish("white" if len(self.__user.get_composed_targets()) is 1 else "grey")
        elif self.__stage is STAGE.stage_pano:
            self.pub_pano.publish("yellow")
            self.pub_orbit.publish("grey")
            self.pub_zigzag.publish("grey")
        elif self.__stage is STAGE.stage_orbit:
            self.pub_pano.publish("grey")
            self.pub_orbit.publish("yellow")
            self.pub_zigzag.publish("grey")
        elif self.__stage is STAGE.stage_zigzag:
            self.pub_pano.publish("grey")
            self.pub_orbit.publish("grey")
            self.pub_zigzag.publish("yellow")
        elif self.__stage is STAGE.stage_restore:
            self.pub_pano.publish("grey")
            self.pub_orbit.publish("grey")
            self.pub_zigzag.publish("grey")
        else:
            self.pub_pano.publish("grey")
            self.pub_orbit.publish("grey")
            self.pub_zigzag.publish("grey")

    def safe_main_routine(self, event):
        try:
            self.main_routine(event)
        except Exception as e:
            print e

    def __best_hover_position(self):
        # if self.__planner.get_hover_position() is None:
        #     return self.__orb.get_position()
        positions = dict()
        for x in range(-4, 5):
            for y in range(-3, 4):
                for z in range(-4, 5):
                    positions[(x,y,z)] = [self.__orb.get_position()[0] + x**2 * (0.25 if x > 0 else -0.25),
                                          self.__orb.get_position()[1] + y**2 * (0.25 if y > 0 else -0.25),
                                          self.__orb.get_position()[2] + z**2 * (0.25 if z > 0 else -0.25)]

        composition_errors = dict()
        for key in positions.keys():
            best_orientations = []
            composition_error = 0

            for target_id in self.__user.get_composed_compositions().keys():
                orientation = self.__target_manager.compute_camera_orientation(positions[key], target_id, self.__user.get_composed_compositions()[target_id])
                if orientation is not None:
                    best_orientations.append(orientation)

            average_pan = np.average(map(lambda orientation: orientation[0], best_orientations))
            average_tilt = np.average(map(lambda orientation: orientation[1], best_orientations))
            average_orientation = [average_pan, average_tilt]

            # if len(best_orientations) is not 0:
            #     print "MAIN: best_orientations[0]", best_orientations[0]
            # print "MAIN: average_orientation", average_orientation

            for target_id in self.__user.get_composed_compositions().keys():
                composition_error += self.__target_manager.compute_composition_error(positions[key], average_orientation, target_id, self.__user.get_composed_compositions()[target_id])

            if len(self.__user.get_composed_compositions()) == 0:
                return self.__orb.get_position()

            composition_errors[key] = composition_error / len(self.__user.get_composed_compositions())

            # if key == (0,0,0):
            #     print "MAIN: (0,0,0) is ", self.__orb.get_position(), "pan tilt is", average_pan, average_tilt
            #     if len(self.__user.get_composed_compositions().keys()) > 0:
            #         print "MAIN: target is", self.__target_manager.get_target(self.__user.get_composed_compositions().keys()[0]).get_center()

        best_key = (0,0,0)
        for key in composition_errors.keys():
            if composition_errors[key] + distance_between(np.array(self.__orb.get_position()), np.array(positions[key])) * ERROR_TOLERANCE_lazy_factor_px_per_meter  \
                < composition_errors[best_key] + distance_between(np.array(self.__orb.get_position()), np.array(positions[best_key])) * ERROR_TOLERANCE_lazy_factor_px_per_meter:
                best_key = key

        # print "MAIN: (0,0,0) is ", self.__orb.get_position(), "error is", composition_errors[(0,0,0)]
        # print "MAIN: error at best", composition_errors[best_key]
# ==================
        hoverPositionMsg = PointCloud()

        channel_error = ChannelFloat32()
        channel_error.name = "composition error"

        hoverPositionMsg.header = self.__orb.get_header()
        hoverPositionMsg.header.frame_id = FRAME_ID_WORLD

        for key in positions.keys():
            hoverPositionMsg.points.append(Point32(positions[key][0], positions[key][1], positions[key][2]))
            channel_error.values.append(composition_errors[key] + distance_between(np.array(self.__orb.get_position()), np.array(positions[key])) * ERROR_TOLERANCE_lazy_factor_px_per_meter)

        hoverPositionMsg.channels.append(channel_error)

        self.__pub_hover_positions.publish(hoverPositionMsg)
#=====================
        return positions[best_key]

    def main_routine(self, event):
        # pub interface look
        self.pub_interface_look()

        # pub debug info
        self.__orb.pub_debug_info()
        self.__planner.pub_debug_info()
        self.__controller.pub_debug_info()

        if self.__is_overtaken:
            print "MAIN: keyboard control", rospy.Time.now().to_time()
            return

        # check connect
        if self.__orb.get_header() is None:
            self.reset_all()
            self.__cmd.send_bebop_hover()
            return
        elif self.__last_header is None:
            self.reset_all()
            self.__last_header = self.__orb.get_header()
            self.__is_last_header_new = True
        else:
            # print "-------------------------------------------------"
            # print "MAIN: last time", self.__last_header.stamp, "nsecs"
            # print "MAIN: current time", self.__orb.get_header().stamp, "nsecs"
            # print "MAIN: duration", self.__orb.get_header().stamp - self.__last_header.stamp, "nsecs"
            if self.__orb.get_header().stamp - self.__last_header.stamp < rospy.Duration(nsecs=10):
                if self.__is_last_header_new:
                    self.__is_last_header_new = False
                    # print "MAIN: connection lost ----> ONCE"
                else:
                    self.__cmd.send_bebop_hover()
                    # print "MAIN: connection lost ----> TWICE ----> Hovering"
                    return
            else:
                self.__last_header = self.__orb.get_header()
                self.__is_last_header_new = True
            # print "-------------------------------------------------"

        # update compositions
        self.__target_manager.update_compositions_and_publish_targets_as_pcl(self.__orb.get_header(),
                                                                             self.__orb.get_world2cam_as_matrix(),
                                                                             self.__user.get_compositions())
        # force user to reset target composition behind camera
        for target_id in self.__user.get_composed_targets():
            if self.__target_manager.get_target_composition(target_id) is None:
                self.__user.set_target_composition(target_id, None)

        # HERE planner and controller are doing their jobs
        if self.__stage is STAGE.stage_init:
            # block by taking off or landing
            pass
        elif self.__stage is STAGE.stage_idle:
            # CONTROLLER: set current state
            tilt, pan = self.__imu.get_tilt_pan()
            self.__controller.set_current_state(self.__orb.get_position(),
                                                self.__orb.get_orientation(),
                                                self.__orb.get_velocity(),
                                                self.__target_manager.get_target_compositions(self.__user.get_composed_targets()),
                                                pan, tilt)

            # PLANNER: set hover position
            if len(self.__user.get_composed_targets()) is 0:
                if self.__planner.get_hover_position() is None:
                    self.__planner.set_hover_position(self.__orb.get_position())
            else:
                self.__planner.set_hover_position(self.__best_hover_position())
            # if self.__planner.get_hover_position() is None:
            #     self.__planner.set_hover_position(self.__orb.get_position())
            if self.__user.get_intended_position_offset() is not None:
                self.__planner.update_hover_position(self.__orb.get_position(),
                                                     self.__user.get_intended_position_offset())
            # CONTROLLER: set target state
            target_image_position = self.__planner.get_hover_position()
            if self.__user.get_intended_position_offset() is not None:
                target_image_position = [self.__planner.get_hover_position()[0] + self.__user.get_intended_position_offset()[0],
                                         self.__planner.get_hover_position()[1] + self.__user.get_intended_position_offset()[1],
                                         self.__planner.get_hover_position()[2] + self.__user.get_intended_position_offset()[2]]
            self.__controller.set_target_state(target_image_position,
                                               [0,0,0],
                                               self.__user.get_composed_compositions(),
                                               self.__user.get_intended_orientation(self.__orb.get_orientation()))

            # compute
            self.__controller.compute_control()

            # control
            self.__cmd.send_cmd_vel(self.__controller.get_control_forward(), self.__controller.get_control_left(), self.__controller.get_control_up(), self.__controller.get_control_turn_left())
            pan, tilt = self.__cmd.set_tilt_pan(self.__controller.get_control_goal_tilt(), self.__controller.get_control_goal_pan())
            self.__imu.set_tilt_pan(tilt, pan)

        elif self.__stage is STAGE.stage_restore:
            # CONTROLLER: set current state
            tilt, pan = self.__imu.get_tilt_pan()
            self.__controller.set_current_state(self.__orb.get_position(),
                                                self.__orb.get_orientation(),
                                                self.__orb.get_velocity(),
                                                self.__target_manager.get_target_compositions(self.__user.get_composed_targets()),
                                                pan, tilt)

            # PLANNER: check restore stage
            if self.__planner.is_restore_done(self.__orb.get_position(),
                                              self.__orb.get_orientation(),
                                              self.__target_manager.get_target_compositions(self.__planner.get_restore_compositions().keys())):
                self.__user.reset_intention([self.__planner.get_restore_orientation()[0], self.__planner.get_restore_orientation()[1]]) # a hacky way to handle orientation after restore
                self.__planner.reset_restore()
                self.__stage = STAGE.stage_idle
                print "MAIN: restore done"
                return

            # CONTROLLER: set target state
            self.__controller.set_target_state(self.__planner.get_restore_position(),
                                               [0,0,0],
                                               self.__planner.get_restore_compositions(),
                                               self.__planner.get_restore_orientation())

            # compute
            self.__controller.compute_control()

            # control
            self.__cmd.send_cmd_vel(self.__controller.get_control_forward(), self.__controller.get_control_left(), self.__controller.get_control_up(), self.__controller.get_control_turn_left())
            pan, tilt = self.__cmd.set_tilt_pan(self.__controller.get_control_goal_tilt(), self.__controller.get_control_goal_pan())
            self.__imu.set_tilt_pan(tilt, pan)

        elif self.__stage is STAGE.stage_pano:
            # CONTROLLER: set current state
            tilt, pan = self.__imu.get_tilt_pan()
            self.__controller.set_current_state(self.__orb.get_position(),
                                                self.__orb.get_orientation(),
                                                self.__orb.get_velocity(),
                                                dict(),                     # while pano, no target is composed
                                                pan, tilt)

            # PLANNER: check pano stage
            if self.__planner.is_pano_current_waypoint_reached(self.__orb.get_position(), self.__orb.get_orientation()):
                self.shoot()
                self.__planner.update_pano_waypoint()
            if self.__planner.is_pano_done():
                self.__planner.reset_pano()
                self.__stage = STAGE.stage_idle
                print "MAIN: pano done"
                return

            # CONTROLLER: set target state
            self.__controller.set_target_state(self.__planner.get_hover_position(),
                                               [0,0,0],
                                               dict(),
                                               self.__planner.get_current_pano_waypoint())

            # compute
            self.__controller.compute_control()

            # control
            self.__cmd.send_cmd_vel(self.__controller.get_control_forward(), self.__controller.get_control_left(), self.__controller.get_control_up(), self.__controller.get_control_turn_left())
            pan, tilt = self.__cmd.set_tilt_pan(self.__controller.get_control_goal_tilt(), self.__controller.get_control_goal_pan())
            self.__imu.set_tilt_pan(tilt, pan)


        # TODO: enable shared control during autopilot
        elif self.__stage is STAGE.stage_orbit:
            # CONTROLLER: set current state
            tilt, pan = self.__imu.get_tilt_pan()
            self.__controller.set_current_state(self.__orb.get_position(),
                                                self.__orb.get_orientation(),
                                                self.__orb.get_velocity(),
                                                self.__target_manager.get_target_compositions(self.__user.get_composed_targets()),
                                                pan, tilt)
            # PLANNER: check orbit stage
            if self.__planner.is_orbit_current_waypoint_reached(self.__orb.get_position(),
                                                                self.__user.get_compositions()[self.__planner.get_target().get_id()],
                                                                self.__target_manager.get_target_composition(self.__planner.get_target().get_id())):
                self.shoot()
                self.__planner.update_orbit_waypoint()

            if self.__planner.is_orbit_done():
                self.__planner.reset_orbit()
                self.__stage = STAGE.stage_idle
                print "MAIN: orbit done"
                return

            # CONTROLLER: set target state
            self.__controller.set_target_state(self.__planner.get_current_orbit_waypoint(),
                                               [0,0,0],
                                               self.__user.get_composed_compositions(),
                                               [None, None])

            # compute
            self.__controller.compute_control()

            # control
            self.__cmd.send_cmd_vel(self.__controller.get_control_forward(), self.__controller.get_control_left(), self.__controller.get_control_up(), self.__controller.get_control_turn_left())
            pan, tilt = self.__cmd.set_tilt_pan(self.__controller.get_control_goal_tilt(), self.__controller.get_control_goal_pan())
            self.__imu.set_tilt_pan(tilt, pan)

        elif self.__stage is STAGE.stage_zigzag:
            # CONTROLLER: set current state
            tilt, pan = self.__imu.get_tilt_pan()
            self.__controller.set_current_state(self.__orb.get_position(),
                                                self.__orb.get_orientation(),
                                                self.__orb.get_velocity(),
                                                self.__target_manager.get_target_compositions(self.__user.get_composed_targets()),
                                                pan, tilt)

            # PLANNER: check zigzag stage
            if self.__planner.is_zigzag_current_waypoint_reached(self.__orb.get_position(),
                                                                 self.__user.get_compositions()[self.__planner.get_target().get_id()],
                                                                 self.__target_manager.get_target_composition(self.__planner.get_target().get_id())):
                self.shoot()
                self.__planner.update_zigzag_waypoint()
            if self.__planner.is_zigzag_done():
                self.__planner.reset_zigzag()
                self.__stage = STAGE.stage_idle
                print "MAIN: zigzag done"
                return

            # CONTROLLER: set target state
            self.__controller.set_target_state(self.__planner.get_current_zigzag_waypoint(),
                                               [0,0,0],
                                               self.__user.get_composed_compositions(),
                                               [None, None])

            # compute
            self.__controller.compute_control()

            # control
            self.__cmd.send_cmd_vel(self.__controller.get_control_forward(), self.__controller.get_control_left(), self.__controller.get_control_up(), self.__controller.get_control_turn_left())
            pan, tilt = self.__cmd.set_tilt_pan(self.__controller.get_control_goal_tilt(), self.__controller.get_control_goal_pan())
            self.__imu.set_tilt_pan(tilt, pan)

    def image_compressed_callback(self, data):
        self.__image_compressed_buffer_lock.acquire()
        self.__image_compressed_buffer = data
        self.__image_compressed_buffer_lock.release()

    # def reset_tracker(self, id):
        # if main_target is unselected, it is up to the user to choose the next main target
        # if self.main_target_id is id:
        #     self.main_target_id = -1
        # self.tld.cancel_target(id)
        # self.tracked_targets[id].reset()

    # def reset_all_trackers(self):
    #     self.reset_tracker(i)

    def cancel_all_targets(self):
        self.__user.cancel_all_targets()

    # def reset_orbit(self):
    #     self.is_orbiting = False
    #     # self.is_reoriented_for_orbiting = False
    #     self.orbiting_count = 0
    #
    # def reset_restore(self):
    #     self.is_restoring = False

    def reset_pan_tilt(self):
        pan, tilt = self.__cmd.set_tilt_pan()
        self.__imu.set_tilt_pan(tilt, pan)

    def reset_all(self):
        self.reset_pan_tilt()
        self.cancel_all_targets()
        self.__stage = STAGE.stage_idle
        self.__user.cancel_all_targets()

    # def reset_all(self):
    #     # self.reset_all_trackers()
    #     self.cancel_all_targets()
    #     self.reset_orbit()
    #     self.reset_restore()
    #     self.reset_pan_tilt()

    # def flight_state_callback(self, data):
    #     self.flight_state = data.data

    # def reset_pan_tilt_callback(self, data):
    #     self.reset_pan_tilt()

    def __change_stage_back_to_idle(self, event):
        self.__stage = STAGE.stage_idle

    def shoot(self):
        self.__pub_snap_shot_on_board.publish(Empty())

        self.__image_compressed_buffer_lock.acquire()
        self.__pub_snap_shot.publish(self.__image_compressed_buffer)
        self.__image_compressed_buffer_lock.release()

        pose_composition_msg = Float32MultiArray()

        # pose
        cam2world = self.__orb.get_cam2world()
        pose_composition_msg.data.append(cam2world.translation.x)
        pose_composition_msg.data.append(cam2world.translation.y)
        pose_composition_msg.data.append(cam2world.translation.z)
        pose_composition_msg.data.append(cam2world.rotation.x)
        pose_composition_msg.data.append(cam2world.rotation.y)
        pose_composition_msg.data.append(cam2world.rotation.z)
        pose_composition_msg.data.append(cam2world.rotation.w)

        # composition
        targets_of_interest = self.__user.get_composed_targets()  # not none composition
        compositions_of_interest = self.__target_manager.get_target_compositions(targets_of_interest)

        for target_id in compositions_of_interest.keys():
            if compositions_of_interest[target_id] is not None and \
                                    0 < compositions_of_interest[target_id][0] < IMAGE_WIDTH and \
                                    0 < compositions_of_interest[target_id][1] < IMAGE_HEIGHT:
                pose_composition_msg.data.append(target_id)
                pose_composition_msg.data.append(compositions_of_interest[target_id][0])
                pose_composition_msg.data.append(compositions_of_interest[target_id][1])

        self.__pub_snap_shot_info.publish(pose_composition_msg)
        print "MAIN: shoot"

    def gesture_callback(self, data):
        if len(data.data) is 1:
            # takeoff
            if int(data.data[0]) is ord('t'):
                flight_state = self.__imu.get_flight_state()
                if int(flight_state) == imu_manager.FLIGHT_STATE.state_landed or int(flight_state) == imu_manager.FLIGHT_STATE.state_emergency:
                    self.__cmd.send_takeoff()
                    self.pub_tl.publish("land")
                    self.reset_all()
                    self.__stage = STAGE.stage_init
                    self.__planner.set_hover_position(None)
                    rospy.Timer(rospy.Duration(CONSTANT_takeoff_land_time), self.__change_stage_back_to_idle, oneshot=True)
            # land
            elif int(data.data[0]) is ord('l'):
                flight_state = self.__imu.get_flight_state()
                if int(flight_state) == imu_manager.FLIGHT_STATE.state_flying or int(flight_state) == imu_manager.FLIGHT_STATE.state_hovering:
                    self.__cmd.send_land()
                    self.pub_tl.publish("takeoff")
                    self.reset_all()
                    self.__stage = STAGE.stage_init
                    self.__planner.set_hover_position(None)
                    rospy.Timer(rospy.Duration(CONSTANT_takeoff_land_time), self.__change_stage_back_to_idle, oneshot=True)

            # # cancel all targets
            # elif int(data.data[0]) is ord('N'):
            #     # self.reset_all_trackers()
            #     self.cancel_all_targets()

            # shoot
            elif int(data.data[0]) is ord('s'):
                self.shoot()

            # finger up
            elif int(data.data[0]) is ord('Q'):
                if self.__stage is STAGE.stage_idle:
                    self.__user.reset_intention(self.__orb.get_orientation())

            # pano
            elif int(data.data[0]) is ord('P'):
                self.__planner.plan_pano(self.__orb.get_orientation())
                self.__stage = STAGE.stage_pano
                print "MAIN: start panoing"
            elif int(data.data[0]) is ord('p'):
                self.__user.reset_intention(self.__planner.get_current_pano_waypoint())     # a hacky way of handling stop pano
                self.__planner.reset_pano()
                self.__stage = STAGE.stage_idle
                print "MAIN: stop panoing"

            # orbiting
            elif int(data.data[0]) is ord('O'):
                target_id = self.__user.get_composed_targets()[0]
                self.__planner.plan_orbit(self.__target_manager.get_target(target_id))
                self.__stage = STAGE.stage_orbit
                # self.pub_orbit.publish("yellow")
                print "MAIN: start orbiting"
            elif int(data.data[0]) is ord('o'):
                self.__planner.reset_orbit()
                self.__stage = STAGE.stage_idle
                # self.pub_orbit.publish("grey")
                print "MAIN: stop orbiting"

            # zigzag
            elif int(data.data[0]) is ord('Z'):
                target_id = self.__user.get_composed_targets()[0]
                self.__planner.plan_zigzag(self.__target_manager.get_target(target_id))
                self.__stage = STAGE.stage_zigzag
                # self.pub_orbit.publish("yellow")
                print "MAIN: start zigzaging"
            elif int(data.data[0]) is ord('z'):
                self.__planner.reset_zigzag()
                self.__stage = STAGE.stage_idle
                # self.pub_orbit.publish("grey")
                print "MAIN: stop zigzaging"

        elif len(data.data) is 2:
            # confirm target with id
            if int(data.data[0]) is ord('V'):
                u, v, w, h = self.__target_manager.get_target_composition(data.data[1])
                print "MAIN: user confirm target", data.data[1], "at", u, v
                self.__user.set_target_composition(data.data[1], [u, v])

            # cancel target with id
            elif int(data.data[0]) is ord('N'):
                if self.__stage is STAGE.stage_idle:
                    self.__user.cancel_target(data.data[1])

            # zoom
            elif int(data.data[0]) is ord('Z'):
                self.__user.set_intention_from_user_control(self.__orb.get_cam2world_as_matrix(), 0, 0, data.data[1] - 1., 0, 0)
                print "MAIN: zoom factor is", data.data[1] - 1.

        elif len(data.data) is 3:
            # move around
            if int(data.data[0]) is ord('i'):
                print "MAIN: user move right", data.data[1], "move down", data.data[2]
                self.__user.set_intention_from_user_control(self.__orb.get_cam2world_as_matrix(), data.data[1], data.data[2], 0, 0, 0)

            # pan tilt
            elif int(data.data[0]) is ord('I'):
                print "MAIN: user pan right", data.data[1], "user tilt down", data.data[2]
                self.__user.set_intention_from_user_control(self.__orb.get_cam2world_as_matrix(), 0, 0, 0, data.data[1], data.data[2])

        elif len(data.data) is 4:
            # compose target by direct manipulation
            if int(data.data[0]) is ord('V'):
                print "MAIN: user compose target", data.data[1], "to", data.data[2], data.data[3]
                self.__user.set_target_composition(data.data[1], [data.data[2], data.data[3]])

        # restore, input data with format ['R', X, Y, Z, rx, ry, rz, rw, [id, u, v]]
        elif int(data.data[0]) is ord('R'):
            position = [data.data[1], data.data[2], data.data[3]]
            orientation = transformations.euler_from_quaternion([data.data[4], data.data[5], data.data[6], data.data[7]], axes='ryxz')
            compositions = dict()
            for i in range((len(data.data) - 8) / 3):
                compositions[data.data[3 * i + 8]] = [data.data[3 * i + 9], data.data[3 * i + 10]]

            self.__planner.plan_restore(position, orientation, compositions)
            self.__stage = STAGE.stage_restore
            print "MAIN: start restoring"

        elif int(data.data[0]) is ord('c'):
            # select target
            min_x = None
            max_x = None
            min_y = None
            max_y = None
            for i in range(len(data.data) / 2):
                if min_x is None:
                    min_x = data.data[2 * i + 1]
                else:
                    min_x = min(min_x, data.data[2 * i + 1])
                if max_x is None:
                    max_x = data.data[2 * i + 1]
                else:
                    max_x = max(max_x, data.data[2 * i + 1])
                if min_y is None:
                    min_y = data.data[2 * i + 2]
                else:
                    min_y = min(min_y, data.data[2 * i + 2])
                if max_y is None:
                    max_y = data.data[2 * i + 2]
                else:
                    max_y = max(max_y, data.data[2 * i + 2])

            new_target = self.__target_manager.add_target(min_x, min_y, max_x-min_x, max_y-min_y,
                                                          self.__orb.get_header(),
                                                          self.__orb.get_world2cam_as_matrix(),
                                                          self.__orb.get_pcl_world())
            if not new_target.is_valid():
                print "MAIN: too few features to construct a target"
                return

            new_target_composition = self.__target_manager.get_target_composition(new_target.get_id())
            # self.__user.set_target_composition(new_target.get_id(), new_target_composition[0], new_target_composition[1])
            self.__user.set_target_composition(new_target.get_id(), None)   # select without composing
            print "MAIN: user select target", new_target.get_id(), "at", new_target_composition[0], new_target_composition[1]

            self.__stage = STAGE.stage_idle


            # if len(data.data) % 2 is 1 and self.image_raw_buffer is not None:
            #     min_x = None
            #     max_x = None
            #     min_y = None
            #     max_y = None
            #     for i in range(len(data.data) / 2):
            #         if min_x is None:
            #             min_x = data.data[2 * i]
            #         else:
            #             min_x = min(min_x, data.data[2 * i])
            #         if max_x is None:
            #             max_x = data.data[2 * i]
            #         else:
            #             max_x = max(max_x, data.data[2 * i])
            #
            #         if min_y is None:
            #             min_y = data.data[2 * i + 1]
            #         else:
            #             min_y = min(min_y, data.data[2 * i + 1])
            #         if max_y is None:
            #             max_y = data.data[2 * i + 1]
            #         else:
            #             max_y = max(max_y, data.data[2 * i + 1])
            #     # publish bb
            #     idle_tracker_id = self.tld.select_target(min_x, min_y, max_x - min_x, max_y - min_y, self.image_raw_buffer)
            #     if idle_tracker_id < 0:
            #         return
            #     self.main_target_id = idle_tracker_id
            #     self.reference_main_target_id = idle_tracker_id
            #     self.reference_pan = self.pan
            #     self.reference_tilt = self.tilt
            #     self.reference_height = self.height
            #     self.reference_composition[idle_tracker_id].u = (min_x + max_x) / 2.0
            #     self.reference_composition[idle_tracker_id].v = (min_y + max_y) / 2.0
            #     self.reset_orbit()

    # def tracked_objects_callback(self, data):
    #     for i in range(NUM_OF_TRACKERS):
    #         self.tracked_targets[i].u = data.data[5*i] + data.data[5*i + 2] / 2.0
    #         self.tracked_targets[i].v = data.data[5*i + 1] + data.data[5*i + 3] / 2.0
    #         self.tracked_targets[i].accuracy = data.data[5*i + 4]

    # def pan_callback(self, data):
    #     if data.data > -8888 and self.pan is None:
    #         self.pan = data.data
            # print "last and current pan", self.last_pan, self.pan

    # def tilt_callback(self, data):
    #     if data.data > -8888 and self.tilt is None:
    #         self.tilt = data.data

    # def height_callback(self, data):
    #     if data.data > -8888:
    #         self.height = data.data

    # def image_raw_callback(self, data):
    #     self.image_raw_buffer = data
    #     self.pub_main_target_id.publish(self.main_target_id)
    #     if self.is_orbiting:
    #         self.send_orbiting()
    #         # print "orbiting"
    #     elif self.is_restoring:
    #         self.send_restoring()
    #     else:
    #         self.send_hover()
    #         # print "hovering"

    # def image_compressed_callback(self, data):
    #     self.image_compressed_buffer_lock.acquire()
    #     self.image_compressed_buffer = data
    #     self.image_compressed_buffer_lock.release()

    # def pose_callback(self, data):
    #     self.pose = data.pose

    # def send_hover(self):
    #     # main target is well-tracked
    #     if self.tracked_targets[self.reference_main_target_id].accuracy > 0.5:
    #          t_x, t_y, t_z, r_x, r_y = hover_wrt_multiple_targets(self.reference_pan, self.reference_tilt, self.reference_height, self.reference_composition,
    #                                                               self.pan, self.tilt, self.height, self.tracked_targets, self.reference_main_target_id)
    #          self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #          self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)
    #          print "send_hover: wrt reference_main_target", self.reference_main_target_id
    #     # main target is not well-tracked, then pick another target as the main target, if non is well-tracked, then auto hover
    #     else:
    #         for i in range(NUM_OF_TRACKERS):
    #             if self.tracked_targets[i].accuracy > 0.5 and not self.reference_composition[i].is_idle():
    #                 t_x, t_y, t_z, r_x, r_y = hover_wrt_multiple_targets(self.reference_pan, self.reference_tilt, self.reference_height, self.reference_composition,
    #                                                                      self.pan, self.tilt, self.height, self.tracked_targets, i)
    #                 self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)
    #                 print "send_hover: reference_main_target", self.reference_main_target_id, "is not well-tracked, target", i, "is used is the reference_main_target"
    #                 return
    #         self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #         self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #         print "send_hover: no target"
    #
    #
    #
    #
    # def send_restoring(self, moving_speed_ratio=0.03, angle_error=3):
    #     if abs(self.pan - self.target_pan) > angle_error:
    #         if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #             t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #             self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.height * (moving_speed_ratio if self.pan < self.target_pan else -moving_speed_ratio) + self.user_left, -t_y + self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #             print "send_restoring: target pan between ", self.target_pan - angle_error, "and", self.target_pan + angle_error ,"current pan is", self.pan
    #         else:
    #             self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #             print "send_restoring: losing track"
    #     elif abs(self.tilt - self.target_tilt) > angle_error:
    #         if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #             t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #             self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.height * (moving_speed_ratio if self.tilt > self.target_tilt else -moving_speed_ratio) + self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #             print "send_restoring: target tilt between ", self.target_tilt - angle_error, "and", self.target_tilt + angle_error ,"current tilt is", self.tilt
    #         else:
    #             self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #             print "send_restoring: losing track"
    #     # composition is the first priority?
    #     elif self.is_target_well_composed():
    #         self.reset_restore()
    #         self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #         self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #         print "send_restoring: DONE!"
    #     else:
    #         if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #             t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #             self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #             print "send_restoring: composing"
    #         else:
    #             self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #             print "send_restoring: losing track during composing"
    #
    # def send_orbiting(self, max_angle=20, num_of_shots_on_each_side=1, moving_speed_ratio=0.03, angle_error=3):
    #     # re orient yaw
    #     if not self.is_reoriented_for_orbiting:
    #         if abs(self.pan) > angle_error:
    #             if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                 t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                 self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, -self.pan * 0.008) # turn yaw
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   # ?????
    #                 print "send_orbiting: reorienting, target pan between", -angle_error, "and", angle_error, "current pan is", self.pan
    #             else:
    #                 self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                 print "send_orbiting: losing track during changing yaw"
    #         elif self.is_target_well_composed():
    #             self.is_reoriented_for_orbiting = True
    #             self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #             self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #             print "send_orbiting: ready to orbit"
    #         else:
    #             if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                 t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                 self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #                 print "send_orbiting: reorientation done but target is NOT well composed"
    #             else:
    #                 self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                 print "send_orbiting: losing track during composing for reorientation, should not happen??"
    #     # ready to orbit
    #     else:
    #         # after last shot
    #         if self.orbiting_count is 2 * num_of_shots_on_each_side + 1:
    #             if abs(self.pan) > angle_error:
    #                 if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                     t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                     self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.height * (moving_speed_ratio if self.pan < 0 else -moving_speed_ratio) + self.user_left, -t_y + self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #                     print "send_orbiting: going back to where orbiting starts, target pan between", -angle_error, "and", angle_error, "current pan is", self.pan
    #                 else:
    #                     self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                     print "send_orbiting: losing track during go back to where orbiting starts"
    #             elif self.is_target_well_composed():
    #                 self.reset_orbit()
    #                 self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                 print "send_orbiting: orbiting DONE!"
    #             else:
    #                 if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                     t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                     self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #                     print "send_orbiting: going back to where orbiting starts but target is NOT well composed"
    #                 else:
    #                     self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                     print "send_orbiting: losing track during composing while going back to where orbiting starts, should not happend??"
    #         # orbiting
    #         else:
    #             next_orbiting_target_pan = max_angle - self.orbiting_count * max_angle / num_of_shots_on_each_side
    #
    #             if abs(self.pan - next_orbiting_target_pan) > angle_error:
    #                 if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                     t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                     self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.height * (moving_speed_ratio if self.pan < next_orbiting_target_pan else -moving_speed_ratio) + self.user_left, -t_y + self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #                     print "send_orbiting: taking shot number", self.orbiting_count+1, "target pan between", next_orbiting_target_pan - angle_error, "and", next_orbiting_target_pan + angle_error, "current pan is", self.pan
    #                 else:
    #                     self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                     print "send_orbiting: losing track during taking shot number", self.orbiting_count+1
    #
    #             # composition is the first priority?
    #             elif self.is_target_well_composed():
    #                 self.orbiting_count += 1
    #                 print "send_orbiting: ------------------------------------------shooting", self.orbiting_count, "at pan", self.pan
    #
    #                 self.image_compressed_buffer_lock.acquire()
    #                 self.pub_image_compressed.publish(self.image_compressed_buffer)
    #                 self.image_compressed_buffer_lock.release()
    #
    #                 pose = Float32MultiArray()
    #                 if self.u is not None and self.v is not None:
    #                     pose.data.append(self.accuracy)
    #                     pose.data.append(self.u)
    #                     pose.data.append(self.v)
    #                     pose.data.append(self.pan)
    #                     pose.data.append(self.tilt)
    #                     pose.data.append(self.height)
    #                 self.pub_pose.publish(pose)
    #                 self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                 self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #             else:
    #                 if self.accuracy > 0.5 and self.target_u is not None and self.target_v is not None:
    #                     t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(self.target_u, self.target_v, self.u, self.v, self.tilt, self.pan)
    #                     self.__cmd.send_cmd_vel(t_z + self.user_forward, -t_x + self.user_left, -t_y + self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + r_x + self.user_tilt_cmd, self.pan + r_y + self.user_pan_cmd)   #?????
    #                     print "send_orbiting: composing during taking shot number", self.orbiting_count+1
    #                 else:
    #                     self.__cmd.send_cmd_vel(self.user_forward, self.user_left, self.user_up, 0)
    #                     self.pan, self.tilt = self.__cmd.set_tilt_pan(self.tilt + self.user_tilt_cmd, self.pan + self.user_pan_cmd)
    #                     print "send_orbting: losing track during composing for shot number", self.orbiting_count+1
    #
    # def is_target_well_composed(self, tolerance_ratio=1.0):
    #     return (self.u - self.target_u) ** 2 + (self.v - self.target_v) ** 2 <= 1000 * tolerance_ratio * tolerance_ratio



if __name__ == '__main__':
    gcs = BEBOP_GCS()
    rospy.spin()