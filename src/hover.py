#!/usr/bin/env python
import numpy as np

from constant import *
from angle import *
import math

from main import TRACKED_TARGET

def rotation_c2e(tilt, pan):
    return np.array([
        [1, 0, 0],
        [0, math.cos(tilt * math.pi / 180.), -math.sin(tilt * math.pi / 180.0)],
        [0, math.sin(tilt * math.pi / 180.), math.cos(tilt * math.pi / 180.0)]
    ]).dot(np.array([
        [math.cos(pan * math.pi / 180.), 0, math.sin(pan * math.pi / 180.)],
        [0, 1, 0],
        [-math.sin(pan * math.pi / 180.), 0, math.cos(pan * math.pi / 180.)]
    ]))

def rotation_e2c(tilt, pan):
    return rotation_c2e(-tilt, -pan)

# input:
# - target features: u, v, yaw, tilt, height
# - current values: current_u, current_v, current_yaw, current_tilt, current_height
# output:
# - t_x, t_y, t_z, r_x, r_y
def hover(u, v, pan, tilt, height, current_u, current_v, current_pan, current_tilt, current_height, z = 1.0):

    # error in feature space [delta_u, delta_v, delta_yaw, delta_tilt, delta_height].T
    error_uvyth_5x1 = np.array([[u - current_u, v - current_v, pan - current_pan, tilt - current_tilt, height - current_height]]).T
    # print "error_uvyth_5x1[:2]", error_uvyth_5x1[:2]

    # it is easy to express jacobain in camera frame for u and v
    # jacobian from control space in camera frame [^{c}T, ^{c}R].T to feature subspace [\dot{u}, \dot{v}].T: [\dot{u}, \dot{v}].T = j_tr2uv_c_2x6.dot([^{c}T, ^{c}R].T)
    j_tr2uv_c_2x6 = np.array([
        [CAMERA_Fx / z, 0, - (u - CAMERA_Cx) / z,
                                                    - (u - CAMERA_Cx) * (v - CAMERA_Cy) / CAMERA_Fy, CAMERA_Fx + (u - CAMERA_Cx)**2 / CAMERA_Fx, - CAMERA_Fx / CAMERA_Fy * (v - CAMERA_Cy)],
        [CAMERA_Fy / z, - (v - CAMERA_Cy) / z, 0,
                                                    - CAMERA_Fy - (v - CAMERA_Cy)**2 / CAMERA_Fy, (u - CAMERA_Cx) * (v - CAMERA_Cy) / CAMERA_Fx, CAMERA_Fy / CAMERA_Fx * (u - CAMERA_Cx)]
    ])
    # print "j_tr2uv_c_2x6", j_tr2uv_c_2x6

    # to express in drone frame, we need to rotate the frame
    # jacobian from control space in drone frame [^{e}T, ^{e}R].T to feature subspace [\dot{u}, \dot{v}].T: [\dot{u}, \dot{v}].T = j_tr2uv_e_2x6.dot([^{e}T, ^{e}R].T)
    j_tr2uv_e_2x6 = np.append(j_tr2uv_c_2x6[:, :3].dot(rotation_e2c(current_tilt, current_pan)), j_tr2uv_c_2x6[:, 3:].dot(rotation_e2c(current_tilt, current_pan)), axis=1)
    # print "j_tr2uv_e_2x6", j_tr2uv_e_2x6
    # print "j_tr2uv_e_2x6[:, 3:5]", j_tr2uv_e_2x6[:, 3:5]

    # inverted jacobian from feature subspace [\dot{u}, \dot{v}].T to control subspace in drone frame [^{e}r_x, ^{e}r_y].T
    j_inv_uv2r_e_2x2 = np.linalg.inv(j_tr2uv_e_2x6[:, 3:5])
    # print "j_inv_uv2r_e_2x2", j_inv_uv2r_e_2x2

    # rotation screw [r_x, r_y].T
    r_x, r_y = (np.array([[GAIN_Kp_Rx, GAIN_Kp_Ry]]).T * j_inv_uv2r_e_2x2.dot(error_uvyth_5x1[:2])).T[0]
    print "r_x, r_y", r_x, r_y

    # to compute translation screw, we assume translation screw does not change u and v, since u and v is handled by rotation screw
    # express rotation screw [^{e}r_x, ^{e}r_y].T by translation screw ^{e}T in drone frame: [^{e}r_x, ^{e}r_y].T = coeff_t2r_e_2x3.dot(^{e}T)
    coeff_t2r_e_2x3 = - j_inv_uv2r_e_2x2.dot(j_tr2uv_e_2x6[:, :3]).dot(rotation_e2c(current_tilt, current_pan))
    print "coeff_t2r_e_2x3", coeff_t2r_e_2x3

    # jacobain from control subspace in drone frame [^{e}T, ^{e}r_x, ^{e}r_y].T to feature subspace [\dot{pan}, \dot{tilt}, \dot{height}].T:
    j_tr2pth_e_3x5 = np.array([
        [0, 0, 0, 0, CONSTANT_pan],
        [0, 0, 0, CONSTANT_tilt, 0],
        [0, CONSTANT_height, 0, 0, 0]])

    # jacobian from control subspace in drone frame ^{e}T to feature subspace [\dot{pan}, \dot{tilt}, \dot{height}].T: [\dot{yaw}, \dot{tilt}, \dot{height}].T = j_t2pth_e_3x3.dot([\dot{pan}, \dot{tilt}, \dot{height}].T)
    j_t2pth_e_3x3 = j_tr2pth_e_3x5[:, :3] - j_tr2pth_e_3x5[:, 3:].dot(coeff_t2r_e_2x3)

    j_t2pth_e_3x3_2 = np.array([
        [0, -CONSTANT_pan, 0],
        [-CONSTANT_tilt, 0, 0],
        [0, 0, CONSTANT_height]
    ]).dot(np.append(coeff_t2r_e_2x3, np.array([[0, 1, 0]]), axis=0))
    print "diff in j_t2pth_e_3x3", j_t2pth_e_3x3_2 - j_t2pth_e_3x3

    # inverted jacobian from feature subspace [\dot{pan}, \dot{tilt}, \dot{height}].T to control subspace in drone frame ^{e}T
    j_inv_pth2t_e_3x3 = np.linalg.inv(j_t2pth_e_3x3)
    print "j_inv_pth2t_e_3x3", j_inv_pth2t_e_3x3

    inv_augmented_coeff = np.linalg.inv(np.append(coeff_t2r_e_2x3, np.array([[0, 1, 0]]), axis=0))
    print "inv_augmented_coeff", inv_augmented_coeff
    j_inv_pth2t_e_3x3_2 = inv_augmented_coeff.dot(np.array([
        [0, -1./CONSTANT_tilt, 0],
        [-1./CONSTANT_pan, 0, 0],
        [0, 0, 1./CONSTANT_height]
    ]))
    print "diff in j_inv_pth2t_e_3x3", j_inv_pth2t_e_3x3_2 - j_inv_pth2t_e_3x3

    # translation screw [t_x, t_y, t_z].T
    t_x, t_y, t_z = (np.array([[GAIN_Kp_Tx, GAIN_Kp_Ty, GAIN_Kp_Tz]]).T * j_inv_pth2t_e_3x3.dot(error_uvyth_5x1[2:])).T[0]

    return t_x, t_y, t_z, r_x, r_y

def hover_wrt_one_target(reference_u, reference_v, current_u, current_v, current_tilt, current_pan):
    # error in feature space [delta_u, delta_v].T
    error_uv_2x1 = np.array([[reference_u - current_u, reference_v - current_v]]).T
    # print "error_uv_2x1", error_uv_2x1

    # it is easy to express jacobian in camera frame for u and v
    # jacobian from control subspace in camera frame ^{c}R to feature subspace [\dot{u}, \dot{v}].T: [\dot{u}, \dot{v}].T = j_r2uv_c_2x3.dot(^{c}R)
    j_r2uv_c_2x3 = np.array([
        [- (reference_u - CAMERA_Cx) * (reference_v - CAMERA_Cy) / CAMERA_Fy, CAMERA_Fx + (reference_u - CAMERA_Cx) ** 2 / CAMERA_Fx, - CAMERA_Fx / CAMERA_Fy * (reference_v - CAMERA_Cy)],
        [- CAMERA_Fy - (reference_v - CAMERA_Cy) ** 2 / CAMERA_Fy, (reference_u - CAMERA_Cx) * (reference_v - CAMERA_Cy) / CAMERA_Fx, CAMERA_Fy / CAMERA_Fx * (reference_u - CAMERA_Cx)]
    ])
    # print "j_r2uv_c_2x3", j_r2uv_c_2x3

    # to express in drone frame, we need to rotate the frame
    # jacobian from control subspace in drone frame ^{e}R to feature subspace [\dot{u}, \dot{v}].T: [\dot{u}, \dot{v}].T = j_r2uv_e_2x3.dot(^{e}R)
    j_r2uv_e_2x3 = j_r2uv_c_2x3.dot(rotation_e2c(current_tilt, current_pan))
    # print "j_r2uv_e_2x3", j_r2uv_e_2x3

    # inverted jacobian from feature subspace [\dot{u}, \dot{v}].T to control subspace in drone frame [^{e}r_x, ^{e}r_y].T
    j_inv_uv2r_e_2x2 = np.linalg.inv(j_r2uv_e_2x3[:,:2])
    # print "j_inv_uv2r_e_2x2", j_inv_uv2r_e_2x2

    # rotation screw [r_x, r_y].T
    r_x, r_y = (np.array([[GAIN_Kp_Rx, GAIN_Kp_Ry]]).T * j_inv_uv2r_e_2x2.dot(error_uv_2x1)).T[0]
    # print "r_x, r_y", r_x, r_y

    return 0, 0, 0, r_x, r_y


def j_tr2uv_c_2x6(u, v, z):
    return np.array([
        [CAMERA_Fx / z, 0, - (u - CAMERA_Cx) / z,
                                                    - (u - CAMERA_Cx) * (v - CAMERA_Cy) / CAMERA_Fy, CAMERA_Fx + (u - CAMERA_Cx)**2 / CAMERA_Fx, - CAMERA_Fx / CAMERA_Fy * (v - CAMERA_Cy)],
        [CAMERA_Fy / z, - (v - CAMERA_Cy) / z, 0,
                                                    - CAMERA_Fy - (v - CAMERA_Cy)**2 / CAMERA_Fy, (u - CAMERA_Cx) * (v - CAMERA_Cy) / CAMERA_Fx, CAMERA_Fy / CAMERA_Fx * (u - CAMERA_Cx)]
    ])

# compute the control screw, given the reference values and current values of the feature vector
# the approach is to separate the translation screw and the rotation screw
# the rotation screw is first computed so that one (main) target is tracked and well-composed. The main target is indicated by main_target_id
# the translation screw is then computed to make sure the rest targets are composed as desirable as possible
def hover_wrt_multiple_targets(reference_pan, reference_tilt, reference_height, reference_composition, current_pan, current_tilt, current_height, current_composition, main_target_id, z=1):
    # compute the rotation screw using the main target
    t_x, t_y, t_z, r_x, r_y = hover_wrt_one_target(reference_composition[main_target_id].u, reference_composition[main_target_id].v,
                                                   current_composition[main_target_id].u, current_composition[main_target_id].v,
                                                   current_tilt, current_pan)
    # compute the delta caused by rotation screw
    new_pan, new_tilt = current_pan + r_y, current_tilt + r_x

    #
    useful_target_ids = []
    for i in range(len(reference_composition)):
        if i is not main_target_id and not reference_composition[i].is_idle() and not current_composition[i].is_idle() and current_composition[i].accuracy > 0.5:
            useful_target_ids.append(i)
    print "useful target ids are", useful_target_ids

    # no other targets except the main target
    if len(useful_target_ids) is 0:
        return t_x, t_y, t_z, r_x, r_y

    # it is easy to express jacobian in camera frame for u and v
    # jacobian from control space in camera frame [^{c}T, ^{c}R].T to feature subspace [\dot{u}, \dot{v}, ...].T: [\dot{u}, \dot{v}, ...].T = j_tr2uv_c_2tx6.dot([^{c}T, ^{c}R].T)
    j_tr2uv_c_2tx6 = np.concatenate([j_tr2uv_c_2x6(reference_composition[i].u, reference_composition[i].v, z=1) for i in useful_target_ids], axis=0)

    # split translation screw and rotation screw
    j_t2uv_c_2tx3 = j_tr2uv_c_2tx6[:,:3]
    j_r2uv_c_2tx3 = j_tr2uv_c_2tx6[:,3:]

    # to express in drone frame, we need to rotate the frame
    # jacobian from control subspace in drone frame ^{e}R to feature subspace [\dot{u}, \dot{v}].T: [\dot{u}, \dot{v}].T = j_r2uv_e_2x3.dot(^{e}R)
    j_t2uv_e_2tx3 = j_t2uv_c_2tx3.dot(rotation_e2c(current_tilt, current_pan))
    j_r2uv_e_2tx3 = j_r2uv_c_2tx3.dot(rotation_e2c(current_tilt, current_pan))

    # error in feature space [delta_u, delta_v,...].T
    error_uv_2tx1 = np.concatenate([[[reference_composition[i].u - current_composition[i].u], [reference_composition[i].v - current_composition[i].v]] for i in useful_target_ids], axis=0)

    # new error caused rotation screw
    error_uv_2tx1_new = error_uv_2tx1 + j_r2uv_e_2tx3.dot(np.array([r_x, r_y, 0]).T)

    # inverted jacobian from feature subspace [\dot{u}, \dot{v}, ...].T to control subspace in drone frame [^{e}t_x, ^{e}t_y, ^{e}t_z].T
    j_inv_uv2r_e_3x2t = np.linalg.pinv(j_t2uv_e_2tx3)

    print j_inv_uv2r_e_3x2t.dot(error_uv_2tx1_new)


