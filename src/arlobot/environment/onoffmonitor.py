#! /usr/bin/env python
from __future__ import print_function

# The purpose of this script is to monitor the on/off signal from the M4-ATX.  The on/off signal is connected to inputs
# on both of the Psoc4 boards (because the Psoc4's are 5v tolerant) and then made available via the I2C bus.
#
# The on/off signal is a pulse that is
#     - sent by the M4-ATX power supply when the power switch is turned on and
#     - sent once a minute after the power switch is turned off.
#
# The Psoc4 detects the pulse and set a bit in the
# Of interested here, is to capture the on/off input when the Raspberry Pi is running so that it can be safely shutdown.
# Additionally, it would be possible to invoke the PowerPi watchdog to turn off (but its not clear if this is necessary
# if the Raspberry Pi has already safely shutdown).
#
# Idea stolen from here: https://www.element14.com/community/docs/DOC-78055/l/adding-a-shutdown-button-to-the-raspberry-pi-b


import sys
import time
import os
sys.path.append("../arlobot_bringup/scripts/hal/hw")
import i2c.py

# Raspberry Pi 2 supports only device 1
I2C_DEVICE = 1
PSOC4_1_I2C_ADDR = 0x08
PSCO4_2_I2C_ADDR = 0x09
PSOC4_STATUS_REG = 6
ONOFF_STATUS_BIT = 0x0002

def OnOffAsserted():
    psoc4_1_reg_value = i2c_bus.ReadUint16(PSOC4_1_I2C_ADDR, PSOC4_STATUS_REG)
    psoc4_2_reg_value = i2c_bus.ReadUint16(PSOC4_2_I2C_ADDR, PSOC4_STATUS_REG)

    return (psoc4_1_reg_value & ONOFF_STATUS_BIT) and (psoc4_2_reg_value & ONOFF_STATUS_BIT)

try:
    i2c_bus = I2CBus(I2C_DEVICE)
except RuntimeError:
    print("Unable to instantiate I2C bus device 1")
    sys.exit(1)

while not OnOffAsserted():
    time.sleep(1)

os.system("sudo shutdown -h now")






