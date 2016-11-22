#!/usr/bin/env python
from __future__ import print_function

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from arlobot_base_node import ArlobotBaseNodeStates
from hal.pc_hal_proxy import PCHALProxy, PCHALProxyError


class ArlobotNodeError(Exception):
    pass


class ArlobotNode():

    NAME = 'arlobot_node'

    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(ArlobotNode.NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(ArlobotNode.NAME)
        rospy.on_shutdown(self.Shutdown)


        rospy.loginfo("Waiting for ArlobotBaseStatus service ...")
        rospy.wait_for_service('ArlobotBaseStatus')
        try:
            arlobot_base_status = rospy.ServiceProxy('ArlobotBaseStatus', Trigger)
            response = arlobot_base_status()
            if not response.success:
                raise ArlobotNodeError("Failed response from ArlobotBaseStatus")
        except rospy.ServiceException as e:
            raise ArlobotNodeError(e)
        rospy.loginfo("Service is running")

        rospy.loginfo("Waiting for PCHALService ...")
        try:
            self._hal_proxy = PCHALProxy()
        except PCHALProxyError:
            raise ArlobotNodeError("Unable to create HAL")
        rospy.loginfo("Service is running")

        # Initialization
        self._last_arlobot_base_node_state = ArlobotBaseNodeStates.STATE_UNKNOWN
        
        # Setup subscriptions
        self._arlobot_base_node_subscriber = rospy.Subscriber("base_status", String, self._arlobot_base_node_callback)
        
        
    def _arlobot_base_node_callback(self, msg):
        if msg.data != self._last_arlobot_base_node_state:
            self._last_arlobot_base_node_state = msg.data
            rospy.loginfo("Arlobot Node received status {} from ArlobotBaseNode".format(str(msg.data)))
            
        if msg.data == ArlobotBaseNodeStates.STATE_INITIAL:
            # This is the first expected state
            pass
        elif msg.data == ArlobotBaseNodeStates.STATE_STARTED:
            # This is good, it means ArlobotBaseNode has started.  Be patient ...
            pass
        elif msg.data == ArlobotBaseNodeStates.STATE_CHECKING:
            # This is good, it means ArlobotBaseNode is communicating with components ...
            pass
        elif msg.data == ArlobotBaseNodeStates.STATE_RUNNING:
            # This is awesome, let's rock 'n roll
            pass
        elif msg.data == ArlobotBaseNodeStates.STATE_TIMEOUT:
            # This is not good.  It means the ArlobotBaseNode has failed to connect to the hardware
            rospy.logwarn("{}: Check ArlobotBaseNode hardware connection.".format(msg.data))
        elif msg.data == ArlobotBaseNodeStates.STATE_SHUTDOWN:
            # Either there is a problem in ArlobotBaseNode or ROS is shutting down
            rospy.logwarn("{}: ArlobotBaseNode is shutting down".format(str(msg.data)))
        else:
            rospy.logwarn("Unknown state {}.  No action taken.".format(str(msg.data)))
        
    def Start(self):
        rospy.loginfo("Arlobot Node has started")

    def Loop(self):
        rospy.spin()
        rospy.logwarn("Arlobot Node: shutdown invoked, exiting")
        
    def Shutdown(self):
        rospy.loginfo("Arlobot Node has shutdown")
        
    
if __name__ == "__main__":
    '''
    The main routine for the ArlobotNode
    Here we do the following:
        * create the ArlobotNode,
        * start the node and loop until there is an error or a shutdown
    '''
    arlobot = ArlobotNode()
    arlobot.Start()
    arlobot.Loop()

