#! /usr/bin/env python

from __future__ import print_function
from i2c import I2CBus, I2CBusError
from psoc4hw import Psoc4Hw, Psoc4HwError

if __name__ == "__main__":
    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        raise I2CBusError("Unable to create I2CBus instance")

    try:
        psoc4 = Psoc4Hw(i2c_bus, Psoc4Hw.LEFT_PSOC4_ADDR)
    except Psoc4HwError:
        print("Failed to create Psoc4Hw instances")

    # Consider creating a console application that can interact with the Psoc4Hw
    # so that basic testing can be repeated on other boards.  Maybe make a test
    # application that be run on all boards

    psoc4.SetSpeed(0)
