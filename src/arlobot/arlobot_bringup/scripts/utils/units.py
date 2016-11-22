#!/usr/bin/env python
from __future__ import print_function

from math import pi

def millimeter_to_meter(millimeter):
    return millimeter / 1000.0


def meter_to_millimeter(millimeter):
    return int(millimeter * 1000)


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

