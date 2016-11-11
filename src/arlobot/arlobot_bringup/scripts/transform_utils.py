#!/usr/bin/env python

""" A couple of handy conversion utilities taken from the turtlebot_node.py script found in the
    turtlebot_node ROS package at:

    http://www.ros.org/wiki/turtlebot_node

"""

import PyKDL
from math import pi

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


def uni2diff(v, w, L, R):
    _2_v = 2 * v
    _w_L = w * L
    _2_R = 2 * R

    l_v = (_2_v - _w_L) / _2_R
    r_v = (_2_v + _w_L) / _2_R

    return l_v, r_v


def diff2uni(l_v, r_v, L, R):
    v = (l_v + r_v) * (R / 2)
    w = (r_v - l_v) * (R / L)

    return v, w

