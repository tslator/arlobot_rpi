#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import math
import rospy
from arlobot_nav_proxy import ArlobotNavProxy, ArlobotNavProxyError

class OutAndBack():
    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('Arlobot Base Node Debug', False)
        if enable_debug:
            rospy.init_node('out_and_back', anonymous=False, log_level=rospy.DEBUG)
        else:
            rospy.init_node('out_and_back', anonymous=False)
        rospy.on_shutdown(self.Shutdown)

        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        try:
            self._nav_proxy = ArlobotNavProxy()
        except ArlobotNavProxyError as e:
            rospy.logfatal("Unable to instantiate ArlobotNavProxy {}".format(e.args))

    def Start(self):
        rospy.loginfo("Starting OutAndBack Node ...")
        self._nav_proxy.Stop()
        rospy.loginfo("OutAndBack Node started")

    def Loop(self):
        rospy.loginfo("Starting OutAndBack Navigation ...")
        # Loop once for each leg of the trip
        for i in range(2):
            self._nav_proxy.Stop()
            rospy.loginfo("Moving forward ...")
            self._nav_proxy.Straight(1.0, 0.2, 0.001)
            self._nav_proxy.Stop()
            rospy.loginfo("Move complete")
            rospy.loginfo("Rotating ...")
            self._nav_proxy.Rotate(math.radians(180), 1.0, 2.5)
            rospy.loginfo("Rotation complete")

        self._nav_proxy.Stop()
        rospy.loginfo("OutAndBack Navigation complete")

    def Shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("OutAndBack Navigation Node shutting down ...")
        self._nav_proxy.Stop()
        rospy.loginfo("OutAndBack Navigation Node shutdown")

if __name__ == '__main__':
    outandback = OutAndBack()
    outandback.Start()
    outandback.Loop()
    outandback.Shutdown()
