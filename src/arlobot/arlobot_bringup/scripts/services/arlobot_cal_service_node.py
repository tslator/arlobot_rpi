#!/usr/bin/env python
from __future__ import print_function

"""
Description
"""

import rospy
from service_node import ServiceNode, ServiceNodeError


class ArlobotCalServiceNodeError(Exception):
    pass


class ArlobotCalServiceNode(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self, 'arlobot_cal_service_node', 'CalService')

    def _startup(self, timeout):
        rospy.loginfo("Starting up {} ...".format(self._service_name))
        rospy.loginfo("Successfully started {}".format(self._service_name))

        return True

    def _run(self):
        pass

    def _shutdown(self):
        rospy.loginfo("Shutting down {} ...".format(self._service_name))
        rospy.loginfo("Successfully shutdown {}".format(self._service_name))

        return True

if __name__ == '__main__':
    nav_service = ArlobotCalServiceNode()
    nav_service.Start()
    nav_service.Run()


