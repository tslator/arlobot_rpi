#! /usr/bin/env python

# This file provides proxy access to the PC Hardware Abstraction Layers (HAL)
# The PC HAL is used to access all hardware drivers that run on the PC.

import rospy
from arlobot_msgs.srv import HALSetFloatArray, HALGetFloatArray, HALGetLaserScan, HALGetImu
from service_proxy import ServiceProxy, ServiceProxyError


class PCHALProxyError(Exception):
    pass


class PCHALProxy(ServiceProxy):

    MIN_TILT_ANGLE = -31.0
    MID_TILT_ANGLE = 0.0
    MAX_TILT_ANGLE = 31.0

    MIN_LED = 0
    MAX_LED = 6
    LED_OFF = MIN_LED
    LED_GREEN = 1
    LED_RED = 2
    LED_ORANGE = 3
    LED_GREEN_BLINK = 4
    LED_ORANGE_BLINK = 5
    LED_RED_ORANGE_BLINK = MAX_LED


    def __init__(self):
        ServiceProxy.__init__(self, 'PCHALService')

        self._add_service('get_voltage', 'PCHALGetVoltage', HALGetFloatArray)

        self._add_service('get_temp', 'PCHALGetVoltage', HALGetFloatArray)
        self._add_service('get_speed', 'PCHALGetSpeed', HALGetFloatArray)
        self._add_service('get_accel', 'PCHALGetAccel', HALGetFloatArray)
        self._add_service('get_tilt', 'PCHALGetTilt', HALGetFloatArray)
        self._add_service('set_tilt', 'PCHALSetTilt', HALSetFloatArray)
        self._add_service('set_led', 'PCHALSetLed', HALSetFloatArray)

    def GetVoltage(self):
        response = self._invoke_service('get_voltage')
        return {'VIN' : response.values[0], 'IGN' : response.values[1], '33V' : response.values[2], '5V' : response.values[3], '12V' : response.values[4]}

    def GetTemp(self):
        response = self._invoke_service('get_temp')
        return {'c' : response.values}

    def GetSpeed(self):
        response = self._invoke_service('get_speed')
        return response.values[0]

    def GetAccel(self):
        response = self._invoke_service('get_accel')
        return {'x' : response.values[0], 'y' : response.values[1], 'z' : response.values[2]}

    def GetTilt(self):
        response = self._invoke_service('get_tilt')
        return {'tilt' : response.values[0], 'status' : response.values[1]}

    def SetTilt(self, angle):
        response = self._invoke_service('set_tilt', angle)
        return response.success

    def SetLed(self, value):
        response = self._invoke_service('set_led', value)
        return response.success
