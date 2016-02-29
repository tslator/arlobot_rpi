#!/usr/bin/env python

import rospy
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
# Import some standard message for reporting power, battery etc
from arlobot_exception import ArlobotError


class PowerSensorError(ArlobotError):
    pass


class PowerSensor:
    def __init__(self):
        pass

    def Publish(self):
        pass