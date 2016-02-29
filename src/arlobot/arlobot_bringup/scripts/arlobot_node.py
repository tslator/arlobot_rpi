#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class ArlobotNodeError(Exception):
    pass


class ArlobotNode():
    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node('arlobot_node', log_level=rospy.DEBUG)
        else:
            rospy.init_node('arlobot_node')
        rospy.on_shutdown(self.Shutdown)
        
        # Initialization
        self._last_arlobot_base_node_state = "UNKNOWN"
        
        # Setup subscriptions
        self._arlobot_base_node_subscriber = rospy.Subscriber("base_status", String, self._arlobot_base_node_callback)
        
        
    def _arlobot_base_node_callback(self, msg):
        if msg.data != self._last_arlobot_base_node_state:
            self._last_arlobot_base_node_state = msg.data
            rospy.loginfo("ArlobotNode received status {} from ArlobotBaseNode".format(str(msg.data)))
            
        if msg.data == "INITIAL":
            # This is the first expected state
            pass
        elif msg.data == "STARTED":
            # This is good, it means ArlobotBaseNode has started.  Be patient ...
            pass
        elif msg.data == "RUNNING":
            # This is awesome, let's rock 'n roll
            pass
        elif msg.data == "TIMEOUT":
            # This is not good.  It means the ArlobotBaseNode has failed to connect to the hardware
            rospy.logwarn("{}: Check ArlobotBaseNode hardware connection.".format(msg.data))
        elif msg.data == "SHUTDOWN":
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

