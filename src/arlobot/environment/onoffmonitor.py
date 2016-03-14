#! /usr/bin/env python
from __future__ import print_function

# The purpose of this script is to allow the Raspberry Pi to shutdown gracefully when the Arlobot power switch is turned
# off.
# Idea stolen from here: https://www.element14.com/community/docs/DOC-78055/l/adding-a-shutdown-button-to-the-raspberry-pi-b
#
# The Arlobot PC uses the M4-ATX power supply which is connected directly to the battery and contains logic to maintain
# power long enough to allow the PC to shutdown gracefully.  This logic is being leveraged for the Raspberry Pi by
# connecting the PowerPi to the 12v rail of the M4-ATX which means the Raspberry Pi will power up/down at the same time as
# the PC.  Additionally, the ATX protocol supports a 3.3v signal which notifies the motherboard when power is being
# removed.
#
# To ensure graceful shutdown on the RaspberryPi a script (below) monitors the On/Off signal from the M4 via a GPIO pin.
# When there is a change on the pin the callback executes the shutdown command.  The script will run from /etc/rc.local.
#
# Script Basics:
#    - Run the script automatically via /etc/rc.local
#    - Register a callback on a pin change event
#    - When there is a pin change event, execute the OS shutdown command
#  Notes:
#       How much debounce is needed?
#       Is there a need to stabilize the input,
#       Should we count the number of pulses then only issue shutdown after a certain number.
#       Need to characterize the On/Off signal
#       Is this a 5v or 3.3v signal?
#

import RPi.GPIO as GPIO
import os
import time

M4_ON_OFF_PIN = 40 # Pin 40, GPIO21

def OnOffChanged(channel):
    '''
    Monitor pin x which is connected to the on/off signal from the M4 power supply.  When there is a change to the input
    this is a notification to shutdown the Raspberry Pi OS
    :return:
    '''
    print("OnOff pin changed: ", GPIO.input(channel))
    #os.system("sudo shutdown -h now")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(M4_ON_OFF_PIN, GPIO.IN)
GPIO.add_event_detect(M4_ON_OFF_PIN, GPIO.BOTH, callback=OnOffChanged, bouncetime=200)

while 1:
    time.sleep(1)


