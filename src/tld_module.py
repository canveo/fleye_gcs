#!/usr/bin/env python

import roslib
roslib.load_manifest('tld_msgs')
import rospy

from std_msgs.msg import Float32, Float32MultiArray
from tld_msgs.msg import BoundingBox, Target


class TLD_MODULE(object):
    def __init__(self):
        rospy.Subscriber('/tld_fps', Float32, self.fps_callback)
        self.pub_fps = rospy.Publisher('fleye/tld_fps', Float32)

        rospy.Subscriber('/tld_tracked_object', BoundingBox, self.tracked_object_callback)
        self.pub_tracked_object = rospy.Publisher('fleye/tld_tracked_object', Float32MultiArray)

    def fps_callback(self, data):
        self.pub_fps.publish(data)

    def tracked_object_callback(self, data):
        bb = Float32MultiArray()
        bb.data.append(data.x)
        bb.data.append(data.y)
        bb.data.append(data.width)
        bb.data.append(data.height)
        bb.data.append(data.confidence)
        self.pub_tracked_object.publish(bb)
