#!/usr/bin/env python

import rospy
from msgs.msg import DriveSpeed

class ArlobotDriveSpeedPublisher:
    def __init__(self, queue_size=10):
        self._publisher = rospy.Publisher('drive_speed', DriveSpeed, queue_size=queue_size)

    def _msg(self, left, right):
        msg = DriveSpeed()
        msg.left = left
        msg.right = right

        return msg

    def Publish(self, left, right):
        self._publisher.publish(self._msg(left, right))

