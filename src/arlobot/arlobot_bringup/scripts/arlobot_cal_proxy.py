#!/usr/bin/env python

from __future__ import print_function

"""
Description:
"""

import rospy
from arlobot_msgs.srv import CalCommand
from services.service_proxy import ServiceProxy, ServiceProxyError


class ArlobotCalProxyError(Exception):
    pass


class ArlobotCalProxy(ServiceProxy):
    def __init__(self):
        ServiceProxy.__init__(self, 'ArlobotCalService')

        self._add_service('cal_motor', 'ArlobotCalMotor', CalCommand)
        self._add_service('cal_left_pid', 'ArlobotCalLeftPid', CalCommand)
        self._add_service('cal_right_pid', 'ArlobotCalRightPid', CalCommand)
        self._add_service('cal_linear', 'ArlobotCalLinear', CalCommand)
        self._add_service('cal_angular', 'ArlobotCalAngular', CalCommand)

    def DoMotorCalibration(self, mode='raw|parsed|data'):
        """
        :description:
        :param mode: raw - means you get back exactly what came over the serial port just as if you connected directly to it
                     parsed - means the response is parsed into sections: command: <value>, response: <dict>
                     data - means only the data is returned
        :return:
        """
        result = self._invoke_service('cal_motor', [CalCommand.CAL_MOTOR, mode])

        # Convert result.response into the requested output (raw|parsed|data)
        response = result.response

        return response

    def DoLeftPidCalibration(self, mode='raw|parsed|data'):
        result = self._invoke_service('cal_left_pid', [CalCommand.CAL_LEFT_PID, mode])
        # Convert result.response into the requested output (raw|parsed|data)
        response = result.response
        return result.response

    def DoRightPidCalibration(self, mode='raw|parsed|data'):
        result = self._invoke_service('cal_right_pid', [CalCommand.CAL_RIGHT_PID, ])
        # Convert result.response into the requested output (raw|parsed|data)
        response = result.response
        return result.response

    def DoLinearCalibration(self, mode='raw|parsed|data'):
        result = self._invoke_service('cal_linear', [CalCommand.CAL_LINEAR, ])
        # Convert result.response into the requested output (raw|parsed|data)
        response = result.response
        return result.response

    def DoAngularCalibration(self, mode='raw|parsed|data'):
        result = self._invoke_service('cal_angular', [CalCommand.CAL_ANGULAR, ])
        # Convert result.response into the requested output (raw|parsed|data)
        response = result.response
        return result.response
