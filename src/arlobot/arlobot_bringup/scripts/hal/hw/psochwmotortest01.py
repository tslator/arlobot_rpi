#! /usr/bin/env python

from __future__ import print_function
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError
import time

if __name__ == "__main__":
    print("psochwmotortest01:")

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

    max_linear_speed = -0.7
    speed_delta = max_linear_speed / 100
    linear_speed = 0
    angular_speed = 0   # r/s

    psoc.SetSpeed(0,0)
    
    for i in range(100):
        psoc.SetSpeed(linear_speed, angular_speed)
        x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
        print("heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(heading, linear_vel, angular_vel))
        
        time.sleep(0.2)
        linear_speed += speed_delta

    for i in range(100):
        psoc.SetSpeed(linear_speed, angular_speed)
        x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
        print("heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(heading, linear_vel, angular_vel))
       
        time.sleep(0.2)
        linear_speed -= speed_delta

    time.sleep(0.5)

    psoc.SetSpeed(0,0)
