#! /usr/bin/env python

# This file provides proxy access to the Base Hardware Abstraction Layers (HAL)
# The Base HAL is used to access all hardware drivers that run on the Base.

import rospy
from std_srvs.srv import Trigger
from arlobot_msgs.srv import HALSetFloatArray, HALGetFloatArray, HALGetLaserScan, HALGetImu


class BaseHALProxyError(Exception):
    pass


class BaseHALProxy:

    def __init__(self, use_stub=True):
        # Create a connection to the HAL Service

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

    def SetSpeed(self, left, right):
        set_speed = rospy.ServiceProxy('BaseHALSetSpeed', HALSetFloatArray)
        response = set_speed([left, right])
        return response.success

    def GetOdometry(self):
        get_odometry = rospy.ServiceProxy('BaseHALGetOdometry', HALGetFloatArray)
        response = get_odometry()
        return list(response.values)

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
        return { 'orientation'      : dict(zip(['x', 'y', 'z', 'w'], response.orientation)), 
	         'linear_accel'     : dict(zip(['x', 'y', 'z'], response.linear_accel)), 
		 'angular_velocity' : dict(zip(['x', 'y', 'z'], response.angular_velocity)),
                 'magnetic_field'   : dict(zip(['x', 'y', 'z'], response.magnetic_field)),
		 'euler'            : dict(zip(['heading', 'yaw', 'roll', 'pitch'], response.euler)),
                 'temperature'      : dict(zip(['f', 'c'], response.temperature))
	       }

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



