#!/usr/bin/env python
from __future__ import print_function

import rospy

class HALProtocolError(Exception):
    pass


class HALProtocol:

    __HAL_STATE_INITIAL = "INITIAL"
    __HAL_STATE_STOPPED = "STOPPED"
    __HAL_STATE_RUNNING = "RUNNING"
    __HAL_STATE_SHUTDOWN = "SHUTDOWN"

    def __init__(self, name):
        self._name = name
        self._state = HALProtocol.__HAL_STATE_INITIAL

    def _init(self):
        """
        Override this function to perform any initialization needed on startup
        :return:
        """
        pass

    def _start(self):
        """
        Override this function to perform any actions needed on startup
        :return:
        """
        pass

    def _stop(self):
        """
        Override this function to perform any actions needed when stopping
        :return:
        """
        pass

    def _shutdown(self):
        """
        Override this function to perform any action needed on shutdown
        :return:
        """
        pass

    def Startup(self):
        """
        Calls to startup are ignore if the HAL is already running.

        The HAL starts life in the INITIAL state and can perform additional initialization outside of the class __init__
        before transitioning to the STOPPED state

        Once in the STOPPED state, the HAL can perform any work necessary prior to transitioning to the RUNNING state,
        e.g., in simulated mode, start the left/right wheel threads.

        Once the HAL transitions to the RUNNING state it can be used by clients
        :return:
        """

        if self._state is HALProtocol.__HAL_STATE_RUNNING:
            rospy.logwarn("Attempt to startup {} while it is running - No action taken".format(self._name))
            return

        # Handle case when state is INITIAL - the shared object state needs to be setup
        if self._state is HALProtocol.__HAL_STATE_INITIAL:

            rospy.loginfo("{} is initializing ...".format(self._name))

            # Called to perform initialization prior to startup.
            self._init()

            rospy.loginfo("{} initialized".format(self._name))

            rospy.loginfo("{} is stopping ...".format(self._name))

            # Called to perform any processing needed for stopping
            self._stop()

            self._state = HALProtocol.__HAL_STATE_STOPPED
            rospy.loginfo("{} is stopped".format(self._name))

        # Handle case when the shared object state is initialized and ready for the next level of startup
        if self._state is HALProtocol.__HAL_STATE_STOPPED:

            # Called to perform any processing needed for startup
            self._start()

            self._state = HALProtocol.__HAL_STATE_RUNNING
            rospy.loginfo("{} is running".format(self._name))

    def Ready(self, startup_shutdown):
        '''
        Checks if the HAL has completed the associated startup or shutdown request
        :param startup_shutdown: indicates whether the check is for startup or shutdown
        :return:
        '''
        if startup_shutdown:
            return self._state == HALProtocol.__HAL_STATE_RUNNING
        else:
            return self._state == HALProtocol.__HAL_STATE_SHUTDOWN

    def Shutdown(self):
        '''
        Allows the HAL to be shutdown.  There is presently no scenario envisioned where the HAL needs to be shutdown
        while the robot continues to operate.  Generally speaking, the HAL's life cycle is tied to the robot's.  However,
        for completeness it is here and it may serve a purpose at some point.  In the meantime, it functions to ensure
        the hardware is kept in a consistent state while debugging.
        :return:
        '''
        rospy.loginfo("{} is stopping ...".format(self._name))

        # Called to perform any processing needed for stopping
        self._stop()

        self._state = HALProtocol.__HAL_STATE_STOPPED
        rospy.loginfo("{} is stopped".format(self._name))

        rospy.loginfo("{} is shutting down ...".format(self._name))

        # Called to perform any processing needed for shutdown
        self._shutdown()

        self._state = HALProtocol.__HAL_STATE_SHUTDOWN
        rospy.loginfo("{} is shutdown".format(self._name))