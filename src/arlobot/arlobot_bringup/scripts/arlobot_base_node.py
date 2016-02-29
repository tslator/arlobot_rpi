#!/usr/bin/env python

import rospy
from arlobot_base_status_pub import ArlobotBaseStatusPublisher


class ArlobotBaseNodeError(Exception):
    pass


class ArlobotBaseNode:

    __LOOP_RATE = 1

    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node('arlobot_base_node', log_level=rospy.DEBUG)
        else:
            rospy.init_node('arlobot_base_node')
        rospy.on_shutdown(self.Shutdown)

        loop_rate_value = rospy.get_param("base_node_loop_rate", ArlobotBaseNode.__LOOP_RATE)
        
        # Initialization
        self._operational_state =  "INITIAL"
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

    def Start(self):
        rospy.loginfo("Arlobot Base Node has started")

        # Note: a little bit of sleep seems to be necessary to ensure that the INITIAL message is received.
        # Not sure why, but there is no harm in delaying the start of this node.
        rospy.sleep(1)

        self._update_state("INITIAL")
        
        # In the future, there might be other things to do that depend on the HAL being up and running

        self._update_state("RUNNING")

        
    def Loop(self):
        while not rospy.is_shutdown():
            self._update_state("RUNNING")
            self._loop_rate.sleep()
        else:
            rospy.logwarn("Arlobot Base Node: shutdown invoked, exiting")
        
    def Shutdown(self):
        # Note: ROS does not guarantee that messages will be sent after shutdown, so we can't be sure that the status
        # message will be received.  However, services are guaranteed to complete before shutdown completes, so the
        # because there is a service call after this message is sent, the message should be received.
        # We're going to try anyway :-)
        self._update_state("SHUTDOWN")


if __name__ == "__main__":
    arlobotbasenode = ArlobotBaseNode()
    arlobotbasenode.Start()
    arlobotbasenode.Loop()

