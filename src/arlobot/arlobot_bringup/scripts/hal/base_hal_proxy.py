#! /usr/bin/env python

# This file provides proxy access to the Base Hardware Abstraction Layers (HAL)
# The Base HAL is used to access all hardware drivers that run on the Base.

import rospy
from service_proxy import ServiceProxy, ServiceProxyError
from arlobot_msgs.srv import HALSetFloatArray, HALGetFloatArray, HALGetLaserScan, HALGetImu#, HALGetCalibration


class BaseHALProxyError(Exception):
    pass


class BaseHALProxy(ServiceProxy):

    def __init__(self):
        ServiceProxy.__init__(self, 'BaseHALService')

        self._add_service('set_speed', 'BaseHALSetSpeed', HALSetFloatArray)
        self._add_service('get_odometry', 'BaseHALGetOdometry', HALGetFloatArray)
        self._add_service('get_ultrasonic', 'BaseHALGetUltrasonic', HALGetFloatArray)
        self._add_service('get_infrared', 'BaseHALGetInfrared', HALGetFloatArray)
        self._add_service('get_laser_scan', 'BaseHALGetLaserScan', HALGetLaserScan)
        self._add_service('get_imu', 'BaseHALGetImu', HALGetImu)
        self._add_service('get_voltage', 'BaseHALGetVoltage', HALGetFloatArray)
        self._add_service('get_current', 'BaseHALGetCurrent', HALGetFloatArray)
        self._add_service('get_temp', 'BaseHALGetTemp', HALGetFloatArray)
        #self._add_service('get_calibration', 'BaseHALGetCalibration', HALGetCalibration)

    def SetSpeed(self, left, right):
        response = self._invoke_service('set_speed', left, right)
        return response.success

    def GetOdometry(self):
        response = self._invoke_service('get_odometry')
        return list(response.values)

    def GetUltrasonic(self):
        response = self._invoke_service('get_ultrasonic')
        return list(response.values)

    def GetInfrared(self):
        response = self._invoke('get_infrared')
        return list(response.values)

    def GetLaserScan(self):
        response = self._invoke('get_laser_scan')
        return list(response.ranges), list(response.intensities), response.time_increment

    def GetImu(self):
        response = self._invoke_service('get_imu')
        return { 'orientation': dict(zip(['x', 'y', 'z', 'w'], response.orientation)),
                 'linear_accel': dict(zip(['x', 'y', 'z'], response.linear_accel)),
                 'angular_velocity': dict(zip(['x', 'y', 'z'], response.angular_velocity)),
                 'magnetic_field': dict(zip(['x', 'y', 'z'], response.magnetic_field)),
                 'euler': dict(zip(['heading', 'yaw', 'roll', 'pitch'], response.euler)),
                 'temperature': dict(zip(['f', 'c'], response.temperature))
               }

    def GetVoltage(self):
        response = self._invoke_service('get_voltage')
        return response.values

    def GetCurrent(self):
        response = self._invoke_service('get_current')
        return response.values[0]

    def GetTemp(self):
        response = self._invoke_service('get_temp')
        return response.values[0]

    def GetCalibration(self):
        #response = self._invoke_service('get_calibration')
        
        return {
            'psoc': {'calibrated': False, #response.psoc_calibrated,
                     'motors': True, #response.psoc_motors,
                     'pids': True, #response.psoc_pids,
                     'linear': True, #response.psoc_linear,
                     'angular': True}, #response.psoc_angular},
            'imu': {'calibrated': True, #response.psoc_calibrated,
                    'sys': True, #response.imu_sys,
                    'gryo': True, #response.imu_gyro,
                    'accel': True, #response.imu_accel,
                    'mag': True} #response.imu_mag
        }

    def GetCalibrated(self):
        cal = self.GetCalibration()
        return cal['psoc']['calibrated'] and cal['imu']['calibrated']


