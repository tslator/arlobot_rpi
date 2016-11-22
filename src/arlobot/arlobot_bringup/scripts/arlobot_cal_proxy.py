#!/usr/bin/env python

from __future__ import print_function

"""
Description:
"""

import rospy
from std_msgs.msgs import String
from services.service_proxy import ServiceProxy, ServiceProxyError


class ArlobotCalProxyError(Exception):
    pass


class ArlobotCalProxy(ServiceProxy):
    def __init__(self):
        ServiceProxy.__init__(self, 'CalProxy')

        self._add_service('do_motor_calibration', 'CalMotor', String)
        self._add_service('do_left_pid_calibration', 'CalLeftPid', String)
        self._add_service('do_right_pid_calibration', 'CalRightPid', String)
        self._add_service('do_linear_calibration', 'CalLinear', String)
        self._add_service('do_angular_calibration', 'CalAngular', String)

    def DoMotorCalibration(self):
        response = self._invoke_service('do_motor_calibration')
        return response.result

    def DoLeftPidCalibration(self):
        response = self._invoke_service('do_left_pid_calibration')
        return response.result

    def DoRightPidCalibration(self):
        response = self._invoke_service('do_right_pid_calibration')
        return response.result

    def DoLinearCalibration(self):
        response = self._invoke_service('do_linear_calibration')
        return response.result

    def DoAngularCalibration(self):
        response = self._invoke_service('do_angular_calibration')
        return response.result
