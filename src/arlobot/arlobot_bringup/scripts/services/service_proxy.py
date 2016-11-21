#!/usr/bin/env python
from __future__ import print_function

"""
Description:
"""

import rospy
from std_srvs.srv import Trigger


class ServiceProxyError(Exception):
    pass


class ServiceProxy:
    def __init__(self, service_name):
        status_trigger = '{}Status'.format(service_name)
        rospy.wait_for_service(status_trigger)
        try:
            service_status = rospy.ServiceProxy(status_trigger, Trigger)
            response = service_status()
            if not response.success:
                raise ServiceProxyError("Failed response from {}".format(status_trigger))
        except rospy.ServiceException as e:
            raise ServiceProxyError(e.args)

        self._services = {}

    def _add_service(self, name, service, msg_type):
        self._services[name] = rospy.ServiceProxy(service, msg_type)

    def _invoke_service(self, name, *args):
        if len(args) > 0:
            return self._services[name](*args)
        else:
            return self._services[name]()

