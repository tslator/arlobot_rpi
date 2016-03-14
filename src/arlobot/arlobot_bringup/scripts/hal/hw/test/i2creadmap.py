from __future__ import print_function
import sys
import os

# In order to invoke this directly as a test, it is necessary to add the parent directory to the path
# before importing modules to test
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from i2c import I2CBus, I2CBusError
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
    
    map_values = i2c1.ReadArray(psoc_addr, 0, 88, 'b', True)
    for value in map_values:
        print("{:x}".format(value))
