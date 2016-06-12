from __future__ import print_function

import math
import time
from threading import RLock
import rospy
from arlobot_exception import ArlobotError
from hal.hal_proxy import BaseHALProxy, BaseHALProxyError
from drive_pid import DrivePid, DrivePidError
from hal.hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from hal.utils import Worker


class ArlobotDifferentialDriveError(ArlobotError):
    pass


class ArlobotDifferentialDrive():

    def __init__(self,
                 track_width,
                 dist_per_count,
                 max_linear_speed,
                 max_angular_speed,
                 drive_sample_rate,
                 pid_params):

        self._TRACK_WIDTH = track_width
        self._DIST_PER_COUNT = dist_per_count
        self._DRIVE_SAMPLE_RATE = drive_sample_rate

        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._heading = 0
        self._x_dist = 0
        self._y_dist = 0
        self._linear_speed = 0
        self._angular_speed = 0

        self._last_left_count = 0
        self._last_right_count = 0
        self._delta_left_count = 0
        self._delta_right_count = 0

        self._left_pid_speed = 0
        self._right_pid_speed = 0
        self._calc_left_speed = 0
        self._calc_right_speed = 0

        self._lock = RLock()
        self._loop_rate = rospy.Rate(drive_sample_rate)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDifferentialDriveError("Unable to create HAL")

        try:
            self._left_pid = DrivePid(pid_params["kp"], pid_params["ki"], pid_params["kd"])
        except DrivePidError:
            raise ArlobotDifferentialDriveError

        try:
            self._right_pid = DrivePid(pid_params["kp"], pid_params["ki"], pid_params["kd"])
        except DrivePidError:
            raise ArlobotDifferentialDriveError

        try:
            self._diff_drive_worker = Worker("Differential Drive", self.__work)
        except:
            raise ArlobotDifferentialDriveError

    def Start(self):
        self._diff_drive_worker.start()

    def Stop(self):
        self._diff_drive_worker.stop()

    def SetSpeed(self, linear_velocity, angular_velocity):
        linear_velocity = max(-self._max_linear_speed, min(linear_velocity, self._max_linear_speed))
        angular_velocity = max(-self._max_angular_speed, min(angular_velocity, self._max_angular_speed))

        left_speed = linear_velocity - angular_velocity*self._TRACK_WIDTH/2
        right_speed = linear_velocity + angular_velocity*self._TRACK_WIDTH/2

        self._left_pid.SetTarget(left_speed)
        self._right_pid.SetTarget(right_speed)

        rospy.loginfo("lv: {:6.2f}, av: {:6.2f}, ls: {:6.2f}, rs: {:6.2f}".format(linear_velocity, angular_velocity, left_speed, right_speed))

    def GetOdometry(self):
        return self._hal_proxy.GetOdometry()

    def __work(self):

        left_input, right_input = self._hal_proxy.GetSpeed()

        left_output = self._left_pid.Update(left_input, self._loop_rate)
        right_output = self._right_pid.Update(right_input, self._loop_rate)

        self._hal_proxy.SetSpeed(left_output, right_output)

        self._loop_rate.sleep()




