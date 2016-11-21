#!/usr/bin/env python
from __future__ import print_function

"""
Description: This module provides the base class for a service node and implements a protocol for Starting, Running and
Shutting down a service container.  Multiple services can be exposed using this container.
"""

import rospy
from std_srvs.srv import Trigger, TriggerResponse

class ServiceNodeError(Exception):
    pass


class ServiceNode:
    __DEFAULT_TIMEOUT = 5

    def __init__(self, node_name, service_name):
        """
        :description: Initializes the Service node
        :param node_name:
        :param service_name:
        """
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(node_name, log_level=rospy.DEBUG)
        else:
            rospy.init_node(node_name)
        rospy.on_shutdown(self.Shutdown)

        self._service_name = service_name
        self._services = {}

    def _startup(self, timeout=5):
        """
        :description: Override this function to provide handling of the startup of the service.
        :return: True if startup is successful; otherwise, False
        """
        return False

    def _run(self):
        """
        :description: Override this function to add support for additional services
        :return: None
        """
        pass

    def _shutdown(self):
        """
        :description: Override this function to shutdown the service
        :return: True if shutdown is successful; otherwise, False
        """
        return False

    def _service_status(self, request):
        '''
        This method is used to determine if the service is available.  It is only a check of the service node and does
        not convey status of any underlying components
        :param timeout:
        :return:
        '''
        return TriggerResponse(success=True, message="")

    def Start(self):
        rospy.loginfo("{} Service Node starting ...".format(self._service_name))
        if not self._startup(ServiceNode.__DEFAULT_TIMEOUT):
            raise ServiceNodeError("Failed to start {}".format(self._service_name))
        rospy.loginfo("{} Service node started".format(self._service_name))

    def Run(self):
        rospy.loginfo("{} Service Node running".format(self._service_name))

        service_key = '{}ServiceStatus'.format(self._service_name)
        self._services[service_key] = rospy.Service(service_key, Trigger, self._service_status)

        self._run()

        rospy.spin()
        rospy.logwarn("{} Service Node: shutdown invoked, exiting".format(self._service_name))

    def Shutdown(self):
        rospy.loginfo("{} Service Node shutting down".format(self._service_name))

        for key in self._services.keys():
            self._services[key].shutdown()
        if not self._shutdown():
            raise ServiceNodeError("Failed to shutdown {}".format(self._service_name))
        rospy.loginfo("{} Service Node is shutdown".format(self._service_name))
