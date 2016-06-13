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

    speed = 200  # mm/s
    wait_time = 5

    print("Clearing the current odometry ...")
    psoc.ClearOdometry()
    x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
    print("x dist: {}, y dist: {}, heading: {}, linear vel: {}, angular vel: {}".format(x_dist, y_dist, heading, linear_vel,
                                                                                        angular_vel))
    print("Odometry clear complete.")

    print("Commanding motors to {} mm/s".format(speed))
    psoc.SetSpeed(speed, speed)
    print("Waiting for motion to complete ...")

    time.sleep(wait_time)

    print("Motion complete")

    print("Reading odometry")
    x_dist, y_dist, heading, linear_vel, angular_vel = tuple(psoc.GetOdometry())
    print("x dist: {}, y dist: {}, heading: {}, linear vel: {}, angular vel: {}".format(x_dist, y_dist, heading, linear_vel,
                                                                                        angular_vel))
