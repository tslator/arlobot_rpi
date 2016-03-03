#!/usr/bin/env python

import rospy
from arlobot_base_status_pub import ArlobotBaseStatusPublisher
from std_srvs.srv import Trigger, TriggerResponse


class ArlobotBaseNodeError(Exception):
    pass


class ArlobotBaseNodeStates:
    STATE_UNKNOWN = "UNKNOWN"
    STATE_INITIAL = "INITIAL"
    STATE_STARTED = "STARTED"
    STATE_RUNNING = "RUNNING"
    STATE_TIMEOUT = "TIMEOUT"
    STATE_SHUTDOWN = "SHUTDOWN"


class ArlobotBaseNode:

    NAME = 'arlobot_base_node'

    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(ArlobotBaseNode.NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(ArlobotBaseNode.NAME)
        rospy.on_shutdown(self.Shutdown)

        loop_rate_value = rospy.get_param("Base Node Loop Rate")
        
        # Initialization
        self._operational_state =  ArlobotBaseNodeStates.STATE_INITIAL
        self._loop_rate = rospy.Rate(loop_rate_value)

        # Subscriptions

        # Subscribes to battery status message received from ArlobotSafetyNode
        #rospy.Subscriber("battery_state", BatteryState, self._safety_callback)

        # Publications

        # Publishes the Base Status message
        self._arlobot_base_status = ArlobotBaseStatusPublisher()

    def _update_state(self, state):
        if self._operational_state != state:
            rospy.loginfo("Publishing {} state".format(state))
        self._operational_state = state
        self._arlobot_base_status.Publish(self._operational_state)

    def _safety_callback(self, message):
        pass

    def _service_status(self, request):
        response = False

        while self._operational_state != ArlobotBaseNodeStates.STATE_RUNNING:
            rospy.loginfo("Waiting for Arlobot Base Node ...")
            rospy.sleep(0.5)
        else:
            response = True
            rospy.loginfo("Arlobot Base Node is running")
        return TriggerResponse(success=response, message="")


    def Start(self):
        rospy.loginfo("Arlobot Base Node has started")


        # Note: a little bit of sleep seems to be necessary to ensure that the INITIAL message is received.
        # Not sure why, but there is no harm in delaying the start of this node.
        rospy.sleep(1)

        self._update_state(ArlobotBaseNodeStates.STATE_INITIAL)
        
        # In the future, there might be other things to do that depend on the HAL being up and running

        # Start the Arlobot Base Node service
        self._service = rospy.Service('ArlobotBaseStatus', Trigger, self._service_status)

        self._update_state(ArlobotBaseNodeStates.STATE_RUNNING)

        
    def Loop(self):
        rospy.loginfo("Arlobot Base Node entering loop")
        while not rospy.is_shutdown():
            self._update_state(ArlobotBaseNodeStates.STATE_RUNNING)
            self._loop_rate.sleep()
        else:
            rospy.logwarn("Arlobot Base Node: shutdown invoked, exiting")
        
    def Shutdown(self):
        # Note: ROS does not guarantee that messages will be sent after shutdown, so we can't be sure that the status
        # message will be received.  However, services are guaranteed to complete before shutdown completes, so the
        # because there is a service call after this message is sent, the message should be received.
        # We're going to try anyway :-)
        self._update_state(ArlobotBaseNodeStates.STATE_SHUTDOWN)


if __name__ == "__main__":
    arlobotbasenode = ArlobotBaseNode()
    arlobotbasenode.Start()
    arlobotbasenode.Loop()

