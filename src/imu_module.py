#!/usr/bin/env python

import roslib
roslib.load_manifest('bebop_msgs')
import rospy
# import numpy as np
# import math

# Import the messages we're interested in sending and receiving
# from sensor_msgs.msg import CompressedImage        # for receiving the video feed
# from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged # roll, pitch, yaw
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged # height
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3CameraStateOrientation #tilt, pan

from std_msgs.msg import Float32
# from std_msgs.msg import Float32MultiArray

# An enumeration of Drone Statuses
# from drone_status import DroneStatus

# from std_msgs.msg import String
# from std_msgs.msg import Float32
# from tum_ardrone.msg._filter_state import filter_state
# from tum_ardrone.msg._autopilot_cmd import autopilot_cmd
# from tum_ardrone.msg._tagged_objects_state import tagged_objects_state

# this class send control commands to bebop/takeoff, land, cmd_vel and camera_control

FREQUENCY = 50 # Hz

class FLIGHT_STATE():
    state_landed    =0
    state_takingoff =1
    state_hovering  =2
    state_flying    =3
    state_landing   =4
    state_emergency =5


class IMU_MODULE(object):
    def __init__(self):
        rospy.Subscriber('/bebop/states/ARDrone3/CameraState/Orientation', Ardrone3CameraStateOrientation, self.tilt_pan_callback)
        self.pub_tilt = rospy.Publisher('fleye/tilt', Float32)
        self.pub_pan = rospy.Publisher('fleye/pan', Float32)
        self.tilt = None
        self.pan = None

        rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/AltitudeChanged', Ardrone3PilotingStateAltitudeChanged, self.height_callback)
        self.pub_height = rospy.Publisher('fleye/height', Float32)
        self.height = None

        rospy.Subscriber('/bebop/states/ARDrone3/PilotingState/AttitudeChanged', Ardrone3PilotingStateAttitudeChanged, self.rpy_callback)
        self.pub_roll = rospy.Publisher('fleye/roll', Float32)
        self.pub_pitch = rospy.Publisher('fleye/pitch', Float32)
        self.pub_yaw = rospy.Publisher('fleye/yaw', Float32)
        self.roll = None
        self.pitch = None
        self.yaw = None

        rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', Ardrone3PilotingStateFlyingStateChanged, self.flight_state_callback)
        self.pub_flight_state = rospy.Publisher('fleye/flightState', Float32)
        self.flight_state = None

        rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, self.battery_callback)
        self.pub_battery = rospy.Publisher('fleye/battery', Float32)
        self.battery = None

        self.timer = rospy.Timer(rospy.Duration(FREQUENCY/1000.), self.send_imu)

    def tilt_pan_callback(self, data):
        self.tilt = data.tilt
        self.pan = data.pan

    def height_callback(self, data):
        self.height = data.altitude

    def rpy_callback(self, data):
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw

    def flight_state_callback(self, data):
        self.flight_state = data.state

    def battery_callback(self, data):
        self.battery = data.percent

    def send_imu(self, event):
        self.pub_tilt.publish(self.tilt if self.tilt is not None else -9999)
        self.pub_pan.publish(self.pan if self.pan is not None else -9999)

        self.pub_height.publish(self.height if self.height is not None else -9999)

        self.pub_roll.publish(self.roll if self.roll is not None else -9999)
        self.pub_pitch.publish(self.pitch if self.pitch is not None else -9999)
        self.pub_yaw.publish(self.yaw if self.yaw is not None else -9999)

        self.pub_flight_state.publish(self.flight_state if self.flight_state is not None else -9999)

        self.pub_battery.publish(self.battery if self.battery is not None else -9999)
