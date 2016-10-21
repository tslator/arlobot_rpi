#! /usr/bin/env python

# This file provides proxy access to the PC Hardware Abstraction Layers (HAL)
# The PC HAL is used to access all hardware drivers that run on the PC.

import rospy
from std_srvs.srv import Trigger
from arlobot_msgs.srv import HALSetFloatArray, HALGetFloatArray, HALGetLaserScan, HALGetImu


class PCHALProxyError(Exception):
    pass


class PCHALProxy:

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
        rospy.wait_for_service('PCHALServiceStatus')
        try:
            hal_service_status = rospy.ServiceProxy('PCHALServiceStatus', Trigger)
            response = hal_service_status()
            if not response.success:
                raise PCHALProxyError("Failed response from PCHALServiceStatus")
        except rospy.ServiceException, e:
            raise PCHALProxyError(e)

        # Note: We explicitly do not call wait_for_service below because once the HAL is determined to be ready via
        # wait_for_service('PCHALServiceStatus'), all of the other services will also be ready

    def GetVoltage(self):
        get_voltage = rospy.ServiceProxy('PCHALGetVoltage', HALGetFloatArray)
        response = get_voltage()
        return {'VIN' : response.values[0], 'IGN' : response.values[1], '33V' : response.values[2], '5V' : response.values[3], '12V' : response.values[4]}

    def GetTemp(self):
        get_temp = rospy.ServiceProxy('PCHALGetVoltage', HALGetFloatArray)
        response = get_temp()
        return {'c' : response.values}

    def GetSpeed(self):
        get_speed = rospy.ServiceProxy('PCHALGetSpeed', HALGetFloatArray)
        response = get_speed()
        return response.values[0]

    def GetAccel(self):
        get_accel = rospy.ServiceProxy('PCHALGetAccel', HALGetFloatArray)
        response = get_accel()
        return {'x' : response.values[0], 'y' : response.values[1], 'z' : response.values[2]}

    def GetTilt(self):
        get_tilt = rospy.ServiceProxy('PCHALGetTilt', HALGetFloatArray)
        response = get_tilt()
        return {'tilt' : response.values[0], 'status' : response.values[1]}

    def SetTilt(self, angle):
        set_tilt = rospy.ServiceProxy('PCHALSetTilt', HALSetFloatArray)
        response = set_tilt([angle,])
        return response.success

    def SetLed(self, value):
        set_led = rospy.ServiceProxy('PCHALSetLed', HALSetFloatArray)
        response = set_led([value,])
        return response.success
