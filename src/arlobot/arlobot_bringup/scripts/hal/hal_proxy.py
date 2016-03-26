#! /usr/bin/env python

# This file provides access to two Hardware Abstraction Layers (HAL): one for the Base and one for the PC.
# The Base HAL is used to access all hardware drivers that run on the Base.  The PC HAL is used to access all hardware
# drivers that run on the PC.

import rospy

from msgs.srv import HALSetFloatArray, HALGetFloatArray, HALGetLaserScan, HALGetImu
from std_srvs.srv import Trigger


class BaseHALProxyError(Exception):
    pass


class BaseHALProxy:

    def __init__(self, use_stub=True):
        # Create a connection to the HAL Service

        # Note: The HAL service is a collection of services which facilitate access to the hardware resources
        # Instead of calling wait_for_service on every service message, is it possible to call wait_for_service
        # on a single service message, specifically, the one service message that guarantees that the HAL has been
        # started, initialized and is ready for operation.  Then, it shouldn't be necessary to call wait_for_service
        # on the balance of the API, yes?
        rospy.wait_for_service('BaseHALServiceStatus')
        try:
            hal_service_status = rospy.ServiceProxy('BaseHALServiceStatus', Trigger)
            response = hal_service_status()
            if not response.success:
                raise BaseHALProxyError("Failed response from BaseHALServiceStatus")
        except rospy.ServiceException, e:
            raise BaseHALProxyError(e)

        # Note: We explicitly do not call wait_for_service below because once the HAL is determined to be ready via
        # wait_for_service('BaseHALServiceStatus'), all of the other services will also be ready

    def SetSpeed(self, speeds):
        set_speed = rospy.ServiceProxy('BaseHALSetSpeed', HALSetFloatArray)
        left_speed = speeds['left']
        right_speed = speeds['right']
        response = set_speed([left_speed, right_speed])
        return response.success

    def SetAccel(self, accels):
        set_accel = rospy.ServiceProxy('BaseHALSetAccel', HALSetFloatArray)
        left_accel = accels['left']
        right_accel = accels['right']
        response = set_accel([left_accel, right_accel])
        return response.success

    def GetCount(self):
        get_count = rospy.ServiceProxy('BaseHALGetCount', HALGetFloatArray)
        response = get_count()
        left_count = response.values[0]
        right_count = response.values[1]
        return {'left' : left_count, 'right' : right_count}

    def GetSpeed(self):
        get_speed = rospy.ServiceProxy('BaseHALGetSpeed', HALGetFloatArray)
        response = get_speed()
        left_speed = response.values[0]
        right_speed = response.values[1]
        return {'left' : left_speed, 'right' : right_speed}

    def GetUltrasonic(self):
        get_ultrasonic = rospy.ServiceProxy('BaseHALGetUltrasonic', HALGetFloatArray)
        response = get_ultrasonic()
        return list(response.values)

    def GetInfrared(self):
        get_infrared = rospy.ServiceProxy('BaseHALGetInfrared', HALGetFloatArray)
        response = get_infrared()
        return list(response.values)

    def GetLaserScan(self):
        get_laser_scan = rospy.ServiceProxy('BaseHALGetLaserScan', HALGetLaserScan)
        response = get_laser_scan()
        return list(response.ranges), list(response.intensities), response.time_increment

    def GetImu(self):
        get_imu = rospy.ServiceProxy('BaseHALGetImu', HALGetImu)
        response = get_imu()
        return { 'accel' : {'x' : response.accel[0], 'y' : response.accel[1], 'z' : response.accel[2]},
                 'mag' : {'x' : response.mag[0], 'y' : response.mag[1], 'z' : response.mag[2]},
                 'temp' : {'f' : response.temp[0], 'c' : response.temp[1]}}

    def GetVoltage(self):
        get_voltage = rospy.ServiceProxy('BaseHALGetVoltage', HALGetFloatArray)
        response = get_voltage()
        return response.values

    def GetCurrent(self):
        get_current = rospy.ServiceProxy('BaseHALGetCurrent', HALGetFloatArray)
        response = get_current()
        return response.values[0]

    def GetTemp(self):
        get_temp = rospy.ServiceProxy('BaseHALGetTemp', HALGetFloatArray)
        response = get_temp()
        return response.values[0]


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

