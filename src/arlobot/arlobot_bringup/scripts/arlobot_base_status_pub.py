#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from arlobot_exception import ArlobotError

class ArlobotBaseStatusPublisherError(ArlobotError):
    pass


class ArlobotBaseStatusPublisher:
    def __init__(self, queue_size=10):
        self._publisher = rospy.Publisher("base_status", String, queue_size=queue_size)

    def _msg(self, state):
        return state

    def Publish(self, state):
        self._publisher.publish(self._msg(state))
