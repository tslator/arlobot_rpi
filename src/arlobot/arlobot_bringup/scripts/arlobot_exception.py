#!/usr/bin/env python

import rospy

class ArlobotError():
    def __init__(self, error):
        rospy.logfatal(error)

