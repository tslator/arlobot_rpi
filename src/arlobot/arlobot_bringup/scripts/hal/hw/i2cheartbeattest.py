from __future__ import print_function

from i2c import I2CBus, I2CBusError
import sys
import time
import math

if __name__ == "__main__":
    print("i2cspeedtest:")
    print("This test reads the heartbeat from the Psoc as fast as possible and displays it on the console")
    print("This is a test of the I2C through put")

    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        print("successfully created i2c bus")
    except I2CBusError:
        print("Error instantiating I2CBus")

    psoc_addr = 0x08
    heartbeat_offset = 66      # heartbeat offset is 66

    for i in range(100):
        value = i2c1.ReadUint32(psoc_addr, heartbeat_offset, True)
        print("Time: {} - Heartbeat: {}".format(time.time(), value))
        time.sleep(0.5)


