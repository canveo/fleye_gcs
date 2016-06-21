#!/usr/bin/env python

import roslib
roslib.load_manifest('bebop_msgs')
import rospy
# import numpy as np
import math

# Import the messages we're interested in sending and receiving
# from sensor_msgs.msg import CompressedImage        # for receiving the video feed
# from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged # roll, pitch, yaw
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged # height
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3CameraStateOrientation #tilt, pan

from std_msgs.msg import Float32

from threading import Lock
# from std_msgs.msg import Float32MultiArray

# An enumeration of Drone Statuses
# from drone_status import DroneStatus

# from std_msgs.msg import String
# from std_msgs.msg import Float32
# from tum_ardrone.msg._filter_state import filter_state
# from tum_ardrone.msg._autopilot_cmd import autopilot_cmd
# from tum_ardrone.msg._tagged_objects_state import tagged_objects_state

# this class send control commands to bebop/takeoff, land, cmd_vel and camera_control

# FREQUENCY = 50 # Hz

class FLIGHT_STATE():
    state_landed    =0
    state_takingoff =1
    state_hovering  =2
    state_flying    =3
    state_landing   =4
    state_emergency =5

class IMU_MANAGER(object):
    def __init__(self):
        rospy.Subscriber('/bebop/states/ARDrone3/CameraState/Orientation', Ardrone3CameraStateOrientation, self.tilt_pan_callback)
        # self.__pub_tilt = rospy.Publisher('fleye/tilt', Float32, queue_size=1)
        # self.__pub_pan = rospy.Publisher('fleye/pan', Float32, queue_size=1)
        self.__tilt = 0
        self.__pan = 0
        self.__is_tilt_pan_set_manually = False
        self.__tilt_pan_lock = Lock()

        rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.height_callback)
        # self.__pub_height = rospy.Publisher('fleye/height', Float32, queue_size=1)
        self.__height = None
        self.__height_lock = Lock()

        # rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/AttitudeChanged', Ardrone3PilotingStateAttitudeChanged, self.rpy_callback)
        # self.pub_roll = rospy.Publisher('fleye/roll', Float32, queue_size=10)
        # self.pub_pitch = rospy.Publisher('fleye/pitch', Float32, queue_size=10)
        # self.pub_yaw = rospy.Publisher('fleye/yaw', Float32, queue_size=10)
        # self.roll = None
        # self.pitch = None
        # self.yaw = None


        rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/FlyingStateChanged', Ardrone3PilotingStateFlyingStateChanged, self.flight_state_callback)
        self.__pub_flight_state = rospy.Publisher('fleye/flightState', Float32, queue_size=1)
        self.__flight_state = FLIGHT_STATE.state_landed
        self.__flight_state_lock = Lock()

        rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, self.battery_callback)
        self.__pub_battery = rospy.Publisher('fleye/battery', Float32, queue_size=10)
        self.__battery = None
        self.__battery_lock = Lock()

    # --------------------------------------------- tilt pan
    def tilt_pan_callback(self, data):
        self.__tilt_pan_lock.acquire()
        if not self.__is_tilt_pan_set_manually:
            self.__tilt = data.tilt * math.pi / 180.
            self.__pan = data.pan * math.pi / 180.
        self.__tilt_pan_lock.release()

    def set_tilt_pan(self, tilt, pan):
        self.__tilt_pan_lock.acquire()
        self.__is_tilt_pan_set_manually = True
        self.__tilt = tilt
        self.__pan = pan
        self.__tilt_pan_lock.release()

    def get_tilt_pan(self):
        self.__tilt_pan_lock.acquire()
        tilt = self.__tilt
        pan = self.__pan
        self.__tilt_pan_lock.release()
        return tilt, pan

    # --------------------------------------------- height
    def height_callback(self, data):
        self.__height_lock.acquire()
        self.__height = data.altitude
        self.__height_lock.release()

    def get_height(self):
        self.__height_lock.acquire()
        height = self.__height
        self.__height_lock.release()
        return height

    # --------------------------------------------- flight state
    def flight_state_callback(self, data):
        self.__flight_state_lock.acquire()
        self.__flight_state = data.state
        self.__flight_state_lock.release()

    def get_flight_state(self):
        self.__flight_state_lock.acquire()
        flight_state = self.__flight_state
        self.__flight_state_lock.release()
        return flight_state

    def publish_flight_state(self):
        self.__flight_state_lock.acquire()
        if self.__pub_flight_state is not None:
            self.__pub_flight_state.publish(self.__flight_state)
        self.__flight_state_lock.release()

    # --------------------------------------------- battery
    def battery_callback(self, data):
        self.__battery_lock.acquire()
        self.__battery = data.percent
        self.__battery_lock.release()

    def get_battery(self):
        self.__battery_lock.acquire()
        battery = self.__battery
        self.__battery_lock.release()
        return battery

    def publish_battery(self):
        self.__battery_lock.acquire()
        if self.__battery is not None:
            self.__pub_battery.publish(self.__battery)
        self.__battery_lock.release()
