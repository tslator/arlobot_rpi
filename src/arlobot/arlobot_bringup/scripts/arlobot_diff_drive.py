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

        # Note: These are pre calculations of constants used in odometry to eliminate needless computation
        self._SAMPLE_METER_PER_SECOND = self._DRIVE_SAMPLE_RATE * self._DIST_PER_COUNT
        self._METER_COUNT_SAMPLE_METER = (self._DIST_PER_COUNT / self._TRACK_WIDTH)


        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDifferentialDriveError("Unable to create HAL")

        try:
            self._left_pid = DrivePid(pid_params["kp"], pid_params["ki"], pid_params["kd"], self._get_left_speed, self._set_left_speed)
        except DrivePidError:
            raise ArlobotDifferentialDriveError

        try:
            self._right_pid = DrivePid(pid_params["kp"], pid_params["ki"], pid_params["kd"], self._get_right_speed, self._set_right_speed)
        except DrivePidError:
            raise ArlobotDifferentialDriveError

        try:
            self._diff_drive_worker = Worker("Differential Drive", self.__work)
        except:
            raise ArlobotDifferentialDriveError

    def _get_left_speed(self):
        '''
        This function is used by the PID to retrieve the current speed
        :return: current left speed
        '''
        return self._calc_left_speed

    def _get_right_speed(self):
        '''
        This function is used by the PID to retrieve the current speed
        :return: current right speed
        '''
        return self._calc_right_speed

    def _update_speed(self):
        self._hal_proxy.SetSpeed({"left" : self._left_pid_speed, "right" : self._right_pid_speed})

    def _set_left_speed(self, pid_output):
        '''
        This function is used apply the PID result to the wheel speed
        :input: PID output
        :return: None
        '''
        #rospy.loginfo("PLS: {:6.2f}, CLS: {:6.2f}, PO: {:6.2f}".format(self._left_pid_speed, self._calc_left_speed, pid_output))
        #self._left_pid_speed = self._calc_left_speed + pid_output
        self._left_pid_speed = pid_output
        self._update_speed()

    def _set_right_speed(self, pid_output):
        '''
        This function is used apply the PID result to the current speed
        :input: PID output
        :return: None
        '''
        #rospy.loginfo("PRS: {:6.2f}, CRS: {:6.2f}, PO: {:6.2f}".format(self._right_pid_speed, self._calc_right_speed, pid_output))
        #self._right_pid_speed = self._calc_right_speed + pid_output
        self._right_pid_speed = pid_output
        self._update_speed()

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

        #rospy.loginfo("lv: {:6.2f}, av: {:6.2f}, ls: {:6.2f}, rs: {:6.2f}".format(linear_velocity, angular_velocity, left_speed, right_speed))

    def GetOdometry(self):
        with self._lock:
            x_dist = self._x_dist
            y_dist = self._y_dist
            heading = self._heading
            linear_speed = self._linear_speed
            angular_speed = self._angular_speed

        return x_dist, y_dist, heading, linear_speed, angular_speed

    def _calc_odometry(self):
        """
        Calculate speed and distances based on the delta time and delta encoder count
        :return:
        """

        # Calculate the delta encoder counts for left/right wheels
        count = self._hal_proxy.GetCount()

        left_count = count["left"]
        right_count = count["right"]

        delta_left_count = left_count - self._last_left_count
        delta_right_count = right_count - self._last_right_count

        self._last_left_count = left_count
        self._last_right_count = right_count

        # Calculate heading, limit heading to -Pi <= heading < Pi, and update
        #                  delta count diff   meter   sample
        # delta heading = ----------------- X ----- X ------
        #                    sample           count   meter
        delta_heading = (delta_right_count - delta_left_count) * self._METER_COUNT_SAMPLE_METER

        # Calculate x/y distance and update
        delta_dist = 0.5 * (delta_left_count + delta_right_count) * self._DIST_PER_COUNT
        delta_x_dist = delta_dist * math.cos(self._heading)
        delta_y_dist = delta_dist * math.sin(self._heading)

        # Update left/right, linear, and angular speeds
        #          count    sample    meter
        # speed = ------- X ------ X ------
        #          sample    sec      count
        left_speed = delta_left_count * self._SAMPLE_METER_PER_SECOND
        right_speed = delta_right_count * self._SAMPLE_METER_PER_SECOND

        linear_speed = ( right_speed + left_speed ) / 2
        angular_speed = (right_speed - left_speed) / self._TRACK_WIDTH

        with self._lock:
            self._heading += delta_heading
            # I'm not sure why this is neccessary.  Ultimately, it prevents the robot from spinning in place beyond
            # +/- PI.
            #self._heading = max(-math.pi, min(self._heading, math.pi))
            self._x_dist += delta_x_dist
            self._y_dist += delta_y_dist
            self._calc_left_speed = left_speed
            self._calc_right_speed = right_speed
            self._linear_speed = max(-self._max_linear_speed, min(linear_speed, self._max_linear_speed))
            self._angular_speed = max(-self._max_angular_speed, min(angular_speed, self._max_angular_speed))

        #rospy.logdebug("Count: {:6.2f}, {:6.2f}; Delta Count: {:6.2f}, {:6.2f}; Delta Dist: {:6.4f}, {:6.4f}; L/R Speed: {:6.4f}, {:6.4f}, L/A Speed: {:6.4f}, {:6.4f}".format(
        #              left_count, right_count,
        #              delta_left_count, delta_right_count,
        #              delta_x_dist, delta_y_dist,
        #              left_speed, right_speed,
        #              linear_speed, angular_speed))


    def __work(self):
        # Update the PID Controller
        self._left_pid.Update()
        self._right_pid.Update()

        # Update speed for pid and odometry calculations
        self._calc_odometry()

        self._loop_rate.sleep()




