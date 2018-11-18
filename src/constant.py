#!/usr/bin/env python

import numpy as np
import math
from numpy.linalg import inv

FRAME_ID_WORLD = "odom"
FRAME_ID_CAMERA = "base_link"

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 368

CAMERA_Fx = 378.263308
CAMERA_Fy = 373.648186
CAMERA_Cx = 321.342736
CAMERA_Cy = 183.469983

# ref: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
CAMERA_MATRIX = np.array([[CAMERA_Fx,   0,          CAMERA_Cx],
                          [0,           CAMERA_Fy,  CAMERA_Cy],
                          [0,           0,          1]])

CAMERA_MATRIX_INV = inv(CAMERA_MATRIX)

CONSTANT_pan = 3.14 / 2.
CONSTANT_tilt = 360. / 2.
CONSTANT_height = -1

# GAIN_Kp_Tx = -0.1
# GAIN_Kp_Ty = -0.1
# GAIN_Kp_Tz = -0.0005
# GAIN_Kp_Rx = -5
# GAIN_Kp_Ry = -5

#TODO: dynimic config
# GAIN_P_left = 0.2
# GAIN_P_forward = 0.2
# GAIN_P_up = 0.5
# GAIN_P_turn_per_degree = 0.005
GAIN_P_turn_per_radian = 0.005 * 180. / math.pi

# GAIN_D_left = 0
# GAIN_D_forward = 0
# GAIN_D_up = 0
# GAIN_D_turn = 0

GAIN_User_pan_radian_per_unit = IMAGE_WIDTH / 20. * math.pi / 180.
GAIN_User_tilt_radian_per_unit = IMAGE_HEIGHT / 20. * math.pi / 180.

GAIN_User_pan_px_per_unit = IMAGE_WIDTH / 10.
GAIN_User_tilt_px_per_unit = IMAGE_HEIGHT / 10.

GAIN_User_swipe = 1

GAIN_P_pan_radian_per_px = 0.01 * math.pi / 180.
GAIN_P_tilt_radian_per_px = 0.01 * math.pi / 180.

GAIN_P_pan_radian_per_radian = 0.1
GAIN_P_tilt_radian_per_radian = 0.1

ERROR_TOLERANCE_Composition_pan = 5   # px
ERROR_TOLERANCE_Composition_tilt = 5  # px

ERROR_TOLERANCE_Control_pan = 5 * math.pi / 180.    # radian
ERROR_TOLERANCE_Control_tilt = 5 * math.pi / 180.   # radian
ERROR_TOLERANCE_Control_yaw = 15. * math.pi / 180.   # radian

ERROR_TOLERANCE_Control_xyz = 0.2                           # unit in orb
ERROR_TOLERANCE_Control_x = ERROR_TOLERANCE_Control_xyz     # unit in orb
ERROR_TOLERANCE_Control_y = ERROR_TOLERANCE_Control_xyz     # unit in orb
ERROR_TOLERANCE_Control_z = 0#ERROR_TOLERANCE_Control_xyz  / 10.   # unit in orb

# control gain for reflexxes
GAIN_R_left = 1./50.
GAIN_R_forward = 1./50.
GAIN_R_up = 1./30.

GCS_LOOP_FREQUENCY = 30.
ORB_SLAM_FREQUENCY = 29.

ORB_R_std = 0.0015
ORB_Q_std = 0.1

CONSTANT_takeoff_land_time = 3  # secs

ERROR_TOLERANCE_distance_ratio_to_target = 0.2
ERROR_TOLERANCE_angle_difference_wrt_target = 7. * math.pi / 180. # radian
ERROR_TOLERANCE_Composition_pan_factor = 3
ERROR_TOLERANCE_Composition_tilt_factor = 3

ERROR_TOLERANCE_hover_factor = 1.5

ERROR_TOLERANCE_lazy_factor_px_per_meter = 30.