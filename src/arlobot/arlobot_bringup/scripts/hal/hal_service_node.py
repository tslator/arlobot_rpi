#! /usr/bin/env python
from __future__ import print_function

import rospy
from std_srvs.srv import Trigger, TriggerResponse


class HALServiceNodeError(Exception):
    pass


class HALServiceNode:
    """
    The HALServiceNode is responsible for startup and shutdown of the HAL (HardwareAbstractionLayer).
    - Upon startup, the HALServer creates an instance of the HAL, invokes HAL startup, and waits for the HAL to report it
    is ready.
    - Once the HAL is ready, the HALServer publishes the state
    - Upon shutdown, the HALServer invokes HAL shutdown and waits for the HAL to report it has shutdown.
    - Once the HAL is shutdown, the HALServer publishes the state (although, the message is not guaranteed to be received
    due to shutdown being invoked)
    """

    __DEFAULT_TIMEOUT = 5

    def __init__(self, node_name, hal_name):
        '''
        Initialize the node
        Create an instance of HAL
        :return:
        '''
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(node_name, log_level=rospy.DEBUG)
        else:
            rospy.init_node(node_name)
        rospy.on_shutdown(self.Shutdown)

        self._hal = None
        self._hal_name = hal_name
        self._timeout_rate = 0
        self._timeout = 0
        self._wait_rate = 0
        self._services = {}

    def _wait(self, startup_shutdown, timeout):
        '''
        Call HAL.Ready and wait for response
        :param timeout:
        :return:
        '''
        ticks = self._timeout_rate * timeout
        while not self._hal.Ready(startup_shutdown) and ticks > 0:
            rospy.loginfo("Waiting for {} ...".format(self._hal_name))
            ticks -= 1
            self._wait_rate.sleep()
        else:
            if ticks <= 0:
                rospy.loginfo("Timeout waiting for {}".format(self._hal_name))
                return False
            else:
                return True

    def _service_status(self, request):
        '''
        This method is used to determine if the HAL service is available.  It is only a check of the service node and
        does not communicate with the HAL
        :param timeout:
        :return:
        '''
        return TriggerResponse(success=True, message="")

    def _startup(self, timeout=__DEFAULT_TIMEOUT):
        '''
        Call HAL.Startup and wait for response
        :param timeout:
        :return:
        '''
        rospy.loginfo("Starting up {} ...".format(self._hal_name))
        self._hal.Startup()
        success = self._wait(True, timeout)

        if (success):
            rospy.loginfo("Successfully started {}".format(self._hal_name))
        else:
            rospy.logfatal("Failed to start {}".format(self._hal_name))

        return success

    def _shutdown(self, timeout=__DEFAULT_TIMEOUT):
        rospy.loginfo("Shutting down {} ...".format(self._hal_name))
        if self._hal:
            self._hal.Shutdown()
        success = self._wait(False, timeout)
        message = ""

        if (success):
            rospy.loginfo("Successfully shutdown {}".format(self._hal_name))
        else:
            rospy.logwarn("Failed to shutdown {}".format(self._hal_name))

        return success

    def _run(self):
        """
        Override this function to add support for additional services
        :return:
        """
        pass

    def Start(self):
        rospy.loginfo("{} Service Node starting ...".format(self._hal_name))
        if not self._startup(5):
            raise HALServiceNodeError("Failed to start {}".format(self._hal_name))
        rospy.loginfo("{} Service node started".format(self._hal_name))

    def Run(self):
        rospy.loginfo("{} Service Node running".format(self._hal_name))
        self._services[self._hal_name+'ServiceStatus'] = rospy.Service(self._hal_name+'ServiceStatus', Trigger, self._service_status)
        self._run()

        rospy.spin()
        rospy.logwarn("{} Service Node: shutdown invoked, exiting".format(self._hal_name))

    def Shutdown(self):
        rospy.loginfo("{} Service Node shutting down".format(self._hal_name))

        for key in self._services.keys():
            self._services[key].shutdown()
        if not self._shutdown():
            raise HALServiceNodeError("Failed to shutdown {}".format(self._hal_name))
        rospy.loginfo("{} Service Node is shutdown".format(self._hal_name))
