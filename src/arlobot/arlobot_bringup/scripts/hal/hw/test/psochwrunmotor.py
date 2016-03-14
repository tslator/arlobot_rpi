#! /usr/bin/env python

from __future__ import print_function
from src.arlobot.arlobot_bringup.scripts.hal.hw.i2c import I2CBus, I2CBusError
from src.arlobot.arlobot_bringup.scripts.hal.hw.psochw import PsocHw, PsocHwError
import time
import sys

if __name__ == "__main__":
    print("psochwrunmotor:")

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
    wait_time = 60

    print("Stopping motors")
    psoc.SetSpeed(0, 0)

    time.sleep(0.1)

    for i in range(100):
        psoc.SetSpeed(linear_speed, angular_speed)
        time.sleep(0.25)

    time.sleep(3)

    print("Motors should have stopped after 2 seconds")
