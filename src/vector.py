#!/usr/bin/env python
import math
import numpy as np

from numpy import linalg as LA

def project_from_to(f, t):
    return np.dot(f, t) / LA.norm(t) * t