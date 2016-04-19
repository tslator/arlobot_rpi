#! /usr/bin/env python
from __future__ import print_function

# The purpose of this script is to allow the Raspberry Pi to shutdown gracefully when the Arlobot power switch is turned
# off.
# Idea stolen from here: https://www.element14.com/community/docs/DOC-78055/l/adding-a-shutdown-button-to-the-raspberry-pi-b
#
# The Arlobot PC uses the M4-ATX power supply which is connected directly to the battery and contains logic to maintain
# power long enough to allow the PC to shutdown gracefully.  This logic is being leveraged for the Raspberry Pi by
# connecting the PowerPi to the 12v rail of the M4-ATX which means the Raspberry Pi will power up/down at the same time as
# the PC.
#
# The PowerPi has an opto input pin that is, by default, designed to turn on the PowerPi when voltage (3-9v) is applied.
# The opto pin can also be monitored from software running on the Raspberry Pi.  The opto input pin is connected to the
# the 5v rail of the Arlobot power distribution which ensures that the opto pin is asserted when M4-ATX powers on which
# causes the PowerPi and RaspberryPi to power up.  And, because the 5v rail is connected to the power switch, the opto
# input pin will 0v when the power switch is turned off allowing a monitor on the RapsberryPi to recognize the change
# and invoke shutdown.
#
# To ensure graceful shutdown on the RaspberryPi a script (below) monitors the opto input pin state via the PowerPi status
# register (bit 2).  The PowerPi is connected to the I2C bus and exports a number of registers including the status
# register.  The script will run from /etc/rc.local.
#
# Script Basics:
#    - Run the script automatically via /etc/rc.local
#    - Open the I2C bus
#    - Read the status register of the both


import sys
import time
import os
import subprocess

# i2cget explained
#   i2c utility for reading i2c devices
#   -y   - default to warning question
#    1   - i2c device 1
#   0x21 - i2c address of the power pi avr device
#    2   - register offset of power pi status register
#    c   - using this option is the only way to make it work

def OptoAsserted():
    result = subprocess.check_output(["i2cget", "-y", "1", "0x21", "2", "c"])
    asserted = int(result.rstrip(), 16) & 0x04
    #print("Opto Asserted: ", asserted)
    return (asserted)


while OptoAsserted():
    time.sleep(1)

os.system("sudo shutdown -h now")
