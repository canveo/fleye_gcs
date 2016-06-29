#!/usr/bin/env python
import math
import numpy as np

from numpy import linalg as LA

from tf import transformations

# c.f. http://www.lighthouse3d.com/tutorials/maths/vector-projection/
def project_from_to(f, t):
    if LA.norm(t) == 0:
        return t
    return np.dot(f, t) / LA.norm(t) * t

def distance_between(a, b):
    return LA.norm(a-b)

def angle_between(a, b):
    return abs(np.arccos(np.clip(np.dot(a, b) / (LA.norm(a) * LA.norm(b)), -1, 1)))

def length_of_vector(v):
    return LA.norm(v)

def rotate(v, angle, direction, center):
    R_4x4 = transformations.rotation_matrix(angle, direction, center)
    return (np.dot(R_4x4[:3,:3], v) + R_4x4[:3,3]).T