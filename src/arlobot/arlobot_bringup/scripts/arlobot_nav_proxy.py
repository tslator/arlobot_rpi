#!/usr/bin/env python

"""
The arlobot_nav_proxy module provides a wrapper interface to the arlobot_nav_service supporting primitive motions that
move the robot and monitor odometry to verify the move
"""

import rospy
from arlobot_msgs.srv import HALSetFloatArray
from service_proxy import ServiceProxy, ServiceProxyError


class ArlobotNavProxyError(Exception):
    pass


class ArlobotNavProxy(ServiceProxy):

    __DEFAULT_TIMEOUT = 10

    def __init__(self):
        """
        Initializes the ArlobotNavProxy
        """
        ServiceProxy.__init__(self, 'NavService')

        self._add_service('position', 'NavPosition', HALSetFloatArray)
        self._add_service('rotate', 'NavRotate', HALSetFloatArray)
        self._add_service('straight', 'NavStraight', HALSetFloatArray)

    def Position(self, distance, linear_velocity, linear_tolerance, angle, angular_velocity, angular_tolerance, timeout=__DEFAULT_TIMEOUT):
        response = self._invoke_service('position', distance, linear_velocity, linear_tolerance, angle, angular_velocity, angular_tolerance, timeout)
        return response.success

    def Rotate(self, angle, velocity, tolerance, timeout=__DEFAULT_TIMEOUT):
        response = self._invoke_service('rotate', 0.0, 0.0, 0.0, angle, velocity, tolerance, timeout)
        return response.success

    def Straight(self, distance, velocity, tolerance, timeout=__DEFAULT_TIMEOUT):
        response = self._invoke_service('straight', distance, velocity, tolerance, 0.0, 0.0, 0.0, timeout)

