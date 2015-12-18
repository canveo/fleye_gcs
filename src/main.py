#!/usr/bin/env python

import roslib
# roslib.load_manifest('bebop_msgs')
import rospy

from std_msgs.msg import Float32MultiArray

import cmd_module, imu_module, tld_module

if __name__ == '__main__':
    rospy.init_node('fleye_gcs', anonymous=True)
    cmd = cmd_module.CMD_MODULE()
    imu = imu_module.IMU_MODULE()
    tld = tld_module.TLD_MODULE()
    rospy.spin()