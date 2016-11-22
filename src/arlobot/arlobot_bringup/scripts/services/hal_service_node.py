#! /usr/bin/env python
from __future__ import print_function

import rospy
from scripts.service_node import ServiceNode, ServiceNodeError


class HALServiceNodeError(Exception):
    pass


class HALServiceNode(ServiceNode):
    """
    The HALServiceNode is responsible for startup and shutdown of the HAL (HardwareAbstractionLayer).
    - Upon startup, the HALServer creates an instance of the HAL, invokes HAL startup, and waits for the HAL to report it
    is ready.
    - Once the HAL is ready, the HALServer publishes the state
    - Upon shutdown, the HALServer invokes HAL shutdown and waits for the HAL to report it has shutdown.
    - Once the HAL is shutdown, the HALServer publishes the state (although, the message is not guaranteed to be received
    due to shutdown being invoked)
    """

    def __init__(self, node_name, service_name):
        '''
        Initialize the node
        Create an instance of HAL
        :return:
        '''
        ServiceNode.__init__(self, node_name, service_name)

        # Note: The _hal is not instantiated in this class but in the subclass
        self._hal = None
        self._timeout_rate = 0
        self._timeout = 0
        self._wait_rate = 0

    def _wait(self, startup_shutdown, timeout):
        '''
        Call HAL.Ready and wait for response
        :param timeout:
        :return:
        '''
        ticks = self._timeout_rate * timeout
        if self._hal:
            while not self._hal.Ready(startup_shutdown) and ticks > 0:
                rospy.loginfo("Waiting for {} ...".format(self._service_name))
                ticks -= 1
                self._wait_rate.sleep()
            else:
                if ticks <= 0:
                    rospy.loginfo("Timeout waiting for {}".format(self._service_name))
                    return False
                else:
                    return True
        else:
            # The most likely reason for this is that the hal was not instantiated and is None
            rospy.logfatal("HAL was not instantiated.")
        return False

    def _startup(self, timeout):
        '''
        Call HAL.Startup and wait for response
        :param timeout:
        :return:
        '''
        rospy.loginfo("Starting up {} ...".format(self._service_name))
        if self._hal:
            self._hal.Startup()
            success = self._wait(True, timeout)

            if (success):
                rospy.loginfo("Successfully started {}".format(self._service_name))
            else:
                rospy.logfatal("Failed to start {}".format(self._service_name))
        else:
            # The most likely reason for this is that the hal was not instantiated and it None
            rospy.logfatal("HAL was not instantiated")
            success = False

        return success

    def _shutdown(self, timeout):
        rospy.loginfo("Shutting down {} ...".format(self._service_name))
        if self._hal:
            self._hal.Shutdown()
        success = self._wait(False, timeout)
        message = ""

        if (success):
            rospy.loginfo("Successfully shutdown {}".format(self._service_name))
        else:
            rospy.logwarn("Failed to shutdown {}".format(self._service_name))

        return success
