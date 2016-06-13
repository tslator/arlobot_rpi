from __future__ import print_function

from i2c import I2CBus, I2CBusError
import sys
import time

if __name__ == "__main__":
    print("i2cmotortest:")
    print("This test will drive the motors at 200 mm/s for 5 seconds then read the odometry.  The following is expected:")
    print("The motors will move 1 meter in the forward direction and be reflected in the x distance.")
    print("The y distance will remain unchanged")
    print("The heading will be 0")
    print("The linear velocity will be 200 mm/s")
    print("The angular velocity will be 0 mm/s")

    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        print("successfully created i2c bus")
    except I2CBusError:
        print("Error instantiating I2CBus")

    speed = 200           # mm/s
    psoc_addr = 0x08      # Psoc I2C device is as 0x08
    control_offset = 0    # register 0 is the control register
    clear_odometry = 0x02 # bit 1 clears odometry
    odom_offset = 14      # odometry offset is 14
    odom_num_bytes = 20   # each odometry field is a float (4 bytes) and there are 5 fields: x/y dist, heading,
                          # linear/angular velocity
    left_cmd_vel_offset = 2
    right_cmd_vel_offset = 4
    wait_time = 5

    print("Clearing the current odometry ...")
    i2c1.WriteUint16(psoc_addr, control_offset, clear_odometry)
    x_dist, y_dist, heading, linear_vel, angular_vel = tuple(i2c1.ReadArray(psoc_addr, odom_offset, odom_num_bytes, 'f'))
    print("x dist: {}, y dist: {}, heading: {}, linear vel: {}, angular vel: {}".format(x_dist, y_dist, heading, linear_vel, angular_vel))
    print("Odometry clear complete.")

    print("Commanding motors to {} mm/s".format(speed))
    i2c1.WriteInt16(psoc_addr, left_cmd_vel_offset, speed)
    i2c1.WriteInt16(psoc_addr, right_cmd_vel_offset, speed)
    print("Waiting for motion to complete ...")

    time.sleep(wait_time)

    print("Motion complete")

    print("Reading odometry")
    x_dist, y_dist, heading, linear_vel, angular_vel = tuple(i2c1.ReadArray(psoc_addr, odom_offset, odom_num_bytes, 'f'))
    print("x dist: {}, y dist: {}, heading: {}, linear vel: {}, angular vel: {}".format(x_dist, y_dist, heading, linear_vel, angular_vel))

