#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_odom_pub import ArlobotOdometryPublisher
from hal.hal_proxy import BaseHALProxy, BaseHALProxyError

#from arlobot_diff_drive import ArlobotDifferentialDrive, ArlobotDifferentialDriveError
import time


class ArlobotDriveNodeError(Exception):
    pass

class ArlobotDriveNode:

    NAME = 'arlobot_drive_node'

    def __init__(self):
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(ArlobotDriveNode.NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(ArlobotDriveNode.NAME)
        rospy.on_shutdown(self.Shutdown)

        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()

        loop_rate = rospy.get_param("Drive Node Loop Rate")
        self._safety_timeout = rospy.get_param("Drive Node Safety Timeout", 0.5)
        self._odom_linear_scale_correction = rospy.get_param("odom_linear_scale_correction", 1.0)
        self._odom_angular_scale_correction = rospy.get_param("odom_angular_scale_correction", 1.0)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDriveNodeError("Unable to create BaseHALProxy")

        self._hal_proxy.SetSpeed(0, 0)

        self._loop_rate = rospy.Rate(loop_rate)
        self._safety_delta_time = rospy.Time.now()
        self._last_twist_time = rospy.Time.now()

        self._last_linear = 0
        self._last_angular = 0
        self._linear_acceleration = rospy.get_param("Drive Node Velocity Profile Linear Accel", 3)
        self._angular_acceleration = rospy.get_param("Drive Node Velocity Profile Angular Accel", 0.1)
        self._time_slice = rospy.get_param("Drive Node Velocity Profile Time Slice", 0.01)
        velocity_profile_rate = rospy.get_param("Drive Node Velocity Profile Rate", 10)
        self._velocity_profile_max_time = rospy.get_param("Drive Node Velocity Profile Max Time", 0.08)
        self._velocity_profile_sample_time = 1.0/velocity_profile_rate

        if self._velocity_profile_max_time < self._velocity_profile_sample_time:
            raise ArlobotDriveNodeError(
                    "Velocity profile sample time {} must be less than velocity profile max time {}"
                    .format(self._velocity_profile_sample_time, self._velocity_profile_max_time))

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def _apply_motion_profile(self, new_linear, new_angular):
        '''
        Calculate the time needed to achieve the
        t = (v - vo)/a, where tmax is 50ms

        :return:
        '''
        linear_delta = new_linear - self._last_linear
        angular_delta = new_angular - self._last_angular
        self._last_linear = new_linear
        self._last_angular = new_angular

        # Set accel or deccel
        linear_accel = self._linear_acceleration
        angular_accel = self._angular_acceleration
        if linear_delta < 0:
            linear_accel = -self._linear_acceleration
        if angular_delta < 0:
            angular_accel = -self._angular_acceleration

        if linear_delta == 0 and angular_delta == 0:
            # What should be do if this happens?
            pass

        linear_time_to_move = linear_delta/linear_accel
        angular_time_to_move = angular_delta/angular_accel

        # limit the move to 80 ms in length.  our throttle is set to 100 ms.
        delta_time = min(max(linear_time_to_move, angular_time_to_move), self._velocity_profile_max_time)
        num_time_increments = delta_time / self._time_slice # 0.01

        linear_increment = linear_delta / num_time_increments
        angular_increment = angular_delta / num_time_increments

        linear = self._last_linear
        angular = self._last_angular

        for i in range(num_time_increments):
            # Note: Since we are capping the delta time to the max between linear and angular, we need to make
            # sure that we only increment as much as needed for each
            if linear <= new_linear:
                linear += linear_increment
            if angular <= new_angular:
                angular += angular_increment

            self._hal_proxy.SetSpeed(linear, angular)
            time.sleep(self._time_slice)


    def _twist_command_callback(self, command):
        '''
        Throttle the twist commands down to 10 Hz and apply a constant acceleration velocity profile
        :param command:
        :return:
        '''
        delta = rospy.Time.now() - self._last_twist_time

        if (delta > self._velocity_profile_sample_time):
            self._last_twist_time = rospy.Time.now()

            self._apply_motion_profile(command.linear.x, command.angular.z)
            #rospy.logwarn("Twist command: {}".format(str(command)))

    def Start(self):
        pass

    def Loop(self):
        while not rospy.is_shutdown():

            # Check that we are receiving Twist commands fast enough; otherwise, stop the motors
            if self._safety_delta_time.secs > self._safety_timeout:
                rospy.logdebug("Safety Timeout invoked: no twist command for {} seconds".format(self._safety_timeout))
                self._hal_proxy.SetSpeed(0, 0)

            odometry = self._hal_proxy.GetOdometry()
            #rospy.logwarn("x: {:6.3f}, y: {:6.3f}, h: {:6.3f}, l: {:6.3f}, a: {:6.3f}".format(odometry['x_dist'],
             #                                                                                 odometry['y_dist'],
             #                                                                                odometry['heading'],
             #                                                                               odometry['linear'],
             #                                                                                 odometry['angular']))

            # adjust linear and angular speed by calibration factor
            odometry['linear'] = odometry['linear'] * self._odom_linear_scale_correction
            odometry['angular'] = odometry['angular'] * self._odom_angular_scale_correction

            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster,
                                           odometry['heading'],
                                           odometry['x_dist'],
                                           odometry['y_dist'],
                                           odometry['linear'],
                                           odometry['angular'])

            self._loop_rate.sleep()

        else:
            rospy.logwarn("Arlobot Drive Node: shutdown invoked, exiting")

    def Shutdown(self):
        rospy.loginfo("Arlobot Drive Node has shutdown")
        self._drive.SetSpeed(0, 0)


if __name__ == "__main__":
    arlobotdrivenode = ArlobotDriveNode()
    arlobotdrivenode.Start()
    arlobotdrivenode.Loop()
