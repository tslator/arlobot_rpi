#! /usr/bin/env python

from __future__ import print_function
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError
import time

if __name__ == "__main__":
    print("psochwmotortest:")

    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
        print("Successfully opened /dev/i2c1")
    except RuntimeError:
        raise I2CBusError("Unable to create I2CBus instance")

    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
        print("Successfully opened psoc")
    except PsocHwError:
        print("Failed to create PsocHw instances")


    for i in range(1000):
        try:
            x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
            print("x: {:.3f}, y dist: {:.3f}, heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
        except PsocHwError:
            print("received invalid odom values")

