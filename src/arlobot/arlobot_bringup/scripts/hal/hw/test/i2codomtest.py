from __future__ import print_function

from src.arlobot.arlobot_bringup.scripts.hal.hw.i2c import I2CBus, I2CBusError
import sys
import time
import math

if __name__ == "__main__":
    print("i2codomtest:")
    print("This test reads the odometry from the Psoc and displays it on the console")

    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        print("successfully created i2c bus")
    except I2CBusError:
        print("Error instantiating I2CBus")

    psoc_addr = 0x08
    odom_offset = 14      # odometry offset is 14
    odom_num_values = 5   # each odometry field is a float (4 bytes) and there are 5 fields: x/y dist, heading,
                          # linear/angular velocity


    for i in range(100):
        x_dist, y_dist, heading, linear_vel, angular_vel = tuple(i2c1.ReadArray(psoc_addr, odom_offset, odom_num_values, 'f'))
        print("x: {:.3f}, y: {:.3f}, h: {:.3f}, l: {:.3f}, a: {:.3f}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
        time.sleep(0.05)
    
