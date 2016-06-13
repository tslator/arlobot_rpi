from __future__ import print_function

import math
import time
from threading import RLock
import rospy
from arlobot_exception import ArlobotError
from hal.hal_proxy import BaseHALProxy, BaseHALProxyError
from drive_pid import DrivePid, DrivePidError
from hal.utils import Worker


class ArlobotDifferentialDriveError(ArlobotError):
    pass


class ArlobotDifferentialDrive():

    def __init__(self,
                 track_width,
                 max_linear_speed,
                 max_angular_speed):

        self._TRACK_WIDTH = track_width
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDifferentialDriveError("Unable to create HAL")

        self._hal_proxy.SetSpeed(0, 0)

    def SetSpeed(self, linear_velocity, angular_velocity):
        linear_velocity = max(-self._max_linear_speed, min(linear_velocity, self._max_linear_speed))
        angular_velocity = max(-self._max_angular_speed, min(angular_velocity, self._max_angular_speed))

        left_speed = linear_velocity - angular_velocity*self._TRACK_WIDTH/2
        right_speed = linear_velocity + angular_velocity*self._TRACK_WIDTH/2

        self._hal_proxy.SetSpeed(left_speed, right_speed)

        rospy.loginfo("lv: {:6.2f}, av: {:6.2f}, ls: {:6.2f}, rs: {:6.2f}".format(linear_velocity, angular_velocity, left_speed, right_speed))

    def GetOdometry(self):
        return self._hal_proxy.GetOdometry()





