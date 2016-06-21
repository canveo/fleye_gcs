#!/usr/bin/env python
import math
import numpy as np

# CW is +tive, CCW is -tive
# def rad_from_to(f, t):
#     rad = t - f
#     while rad > 2 * math.pi:
#         rad -= 2 * math.pi
#     while rad < -2 * math.pi:
#         rad += 2 * math.pi
#     if abs(rad) > math.pi:
#         rad = (2 * math.pi - abs(rad)) * (-1 if rad > 0 else 1)
#     return rad

def rad_from_to(f, t):
    rad = t - f
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def degree_from_to(f, t):
    degree = t - f
    while degree > 180:
        degree -= 360.
    while degree < -180.:
        degree += 360.
    return degree

def pan_tilt_from_u_v(u, v):
    C_inv = np.array([[ 0.00264366,  0.        , -0.8495213 ],
                      [ 0.        ,  0.00267631, -0.49102335],
                      [ 0.        ,  0.        ,  1.        ]])

    uv_c = np.array([[u],
                     [v],
                     [1]])

    xy_c = C_inv.dot(uv_c)

    return math.atan2(xy_c[0][0],1) / math.pi * 180, -math.atan2(xy_c[1][0], 1) / math.pi * 180