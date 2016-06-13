#! /usr/bin/env python

from __future__ import print_function
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError

if __name__ == "__main__":
    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        raise I2CBusError("Unable to create I2CBus instance")

    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
    except PsocHwError:
        print("Failed to create PsocHw instances")

    # Consider creating a console application that can interact with the PsocHw
    # so that basic testing can be repeated on other boards.  Maybe make a test
    # application that be run on all boards

    psoc.SetSpeed(0)
