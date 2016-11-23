#!/usr/bin/env python

"""
The arlobot_nav_proxy module provides a wrapper interface to the arlobot_nav_service supporting primitive motions that
move the robot and monitor odometry to verify the move
"""

import rospy
from arlobot_msgs.srv import HALSetFloatArray
from services.service_proxy import ServiceProxy, ServiceProxyError


class ArlobotNavProxyError(Exception):
    pass


class ArlobotNavProxy(ServiceProxy):

    __DEFAULT_TIMEOUT = 10

    def __init__(self, timeout=__DEFAULT_TIMEOUT):
        """
        Initializes the ArlobotNavProxy
        """
        ServiceProxy.__init__(self, 'ArlobotNavService')

        print("Adding service calls ...")
        self._add_service('position', 'ArlobotNavPosition', HALSetFloatArray)
        self._add_service('rotate', 'ArlobotNavRotate', HALSetFloatArray)
        self._add_service('straight', 'ArlobotNavStraight', HALSetFloatArray)
        print("Done")

    def Position(self, distance, linear_velocity, linear_tolerance, angle, angular_velocity, angular_tolerance, timeout=__DEFAULT_TIMEOUT):
        print("invoking position service ...")
        response = self._invoke_service('position', [distance, linear_velocity, linear_tolerance, angle, angular_velocity, angular_tolerance, timeout])
        print("done: {}".format(str(response.success)))
        return response.success

    def Rotate(self, angle, velocity, tolerance, timeout=__DEFAULT_TIMEOUT):
        print("invoking rotate service ...")
        response = self._invoke_service('rotate', [angle, velocity, tolerance, timeout])
        print("done: {}".format(str(response.success)))
        return response.success

    def Straight(self, distance, velocity, tolerance, timeout=__DEFAULT_TIMEOUT):
        print("invoking straight service ...")
        response = self._invoke_service('straight', [distance, velocity, tolerance, timeout])
        print("done: {}".format(str(response.success)))
        return response.success

    def Stop(self):
        print("invoking stop service ...")
        response = self._invoke_service('straight', [0.0, 0.0, 0.0, ArlobotNavProxy.__DEFAULT_TIMEOUT])
        print("done: {}".format(str(response.success)))
        return response.success


if __name__ == "__main__":
    try:
        print("starting arlobot nav proxy")
        anp = ArlobotNavProxy()
    except ArlobotNavProxyError as e:
        print("Failed to instantiate ArlobotNavProxy: {}".format(e.args))
        import sys
        sys.exit(1)

    print("calling stop")
    anp.Stop()

    print("Done")
