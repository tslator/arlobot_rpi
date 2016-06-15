#! /usr/bin/env python

from __future__ import print_function
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError
import time

if __name__ == "__main__":
    print("psochwmotortest:")
    print("This test will drive the motors at 200 mm/s for 5 seconds then read the odometry.  The following is expected:")
    print("The motors will move 1 meter in the forward direction and be reflected in the x distance.")
    print("The y distance will remain unchanged")
    print("The heading will be 0")
    print("The linear velocity will be 200 mm/s")
    print("The angular velocity will be 0 mm/s")

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

    linear_speed = 0.2  # m/s
    angular_speed = 0   # r/s
    wait_time = 5

    print("Stopping motors")
    psoc.SetSpeed(0, 0)

    print("Commanding motors to {} m/s".format(linear_speed))
    psoc.SetSpeed(linear_speed, angular_speed)
    print("Waiting for motion to complete ...")

    time.sleep(2.0)

    invalid_odom = True
    while invalid_odom:
        try:
            x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
            print("x dist: {:.3f}, y dist: {:.3f}, heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
            invalid_odom = False
        except PsocHwError, e:
            print("Invalid odometry: {}".format(e))

    
    for i in range(wait_time*100):
        time.sleep(0.01)
        invalid_odom = True
        while invalid_odom:
            try:
                x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
                print("x dist: {:.3f}, y dist: {:.3f}, heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
                invalid_odom = False
            except PsocHwError, e:
                print("Invalid odometry: {}".format(e))


    print("Motion complete")

    psoc.SetSpeed(0,0)


    invalid_odom = True
    while invalid_odom:
        try:
            x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
            print("x dist: {:.3f}, y dist: {:.3f}, heading: {:.3f}, linear vel: {:.3f}, angular vel: {:.3f}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
            invalid_odom = False
        except PsocHwError, e:
            print("Invalid odometry: {}".format(e))


