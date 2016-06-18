#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_odom_pub import ArlobotOdometryPublisher
from hal.hal_proxy import BaseHALProxy, BaseHALProxyError

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

        self._last_linear_velocity = 0
        self._last_angular_velocity = 0
        self._linear_accel = rospy.get_param("Drive Node Velocity Profile Linear Accel", 3)
        self._angular_accel = rospy.get_param("Drive Node Velocity Profile Angular Accel", 0.1)
        self._linear_accel = 10
        self._angular_accel = 1

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def _apply_accel_profile(self, new_linear, new_angular):
        '''
        Apply an acceleration profile to the motion
        :param new_linear:
        :param new_angular:
        :return:
        '''

        linear_delta = new_linear - self._last_linear_velocity
        angular_delta = new_angular - self._last_angular_velocity

        if abs(linear_delta) < 0.001:
            linear_delta = 0
        if abs(angular_delta) < 0.001:
            angular_delta = 0

        linear_time = abs(linear_delta) / self._linear_accel
        angular_time = abs(angular_delta) / self._angular_accel

        linear_accel = self._linear_accel
        if new_linear < self._last_linear_velocity:
            linear_accel = -linear_accel

        angular_accel = self._angular_accel
        if new_angular < self._last_angular_velocity:
            angular_accel = -angular_accel

        velocity_time = max(linear_time, angular_time)
        time_increment = velocity_time / 20
        linear_vel_incr = linear_delta / 20
        angular_vel_incr = angular_delta / 20

        rospy.logwarn("nl {}, na{}, lt {}, at {}, vt {}, ti {}".format(new_linear, new_angular, linear_time, angular_time, velocity_time, time_increment))

        linear = self._last_linear_velocity
        angular = self._last_angular_velocity
        delta_time = 0

        while delta_time < velocity_time:
            if linear < new_linear:
                linear = linear + linear_accel * delta_time
            if angular < new_angular:
                angular = angular + angular_accel * delta_time
            self._hal_proxy.SetSpeed(linear, angular)
            rospy.logwarn("profile: {:6.3f}, {:6.3f}".format(linear, angular))
            time.sleep(delta_time)
            delta_time += time_increment
        self._hal_proxy.SetSpeed(new_linear, new_angular)

        self._last_linear_velocity = new_linear
        self._last_angular_velocity = new_angular

    def _twist_command_callback(self, command):
        '''

        :param command:
        :return:
        '''

        # Think about applying an acceleration profile here:
        #   v = v0 + at
        #
        # Where we specify the acceleration as a parameter and therefore limit how quickly the velocity will change
        #

        #self._apply_accel_profile(command.linear.x, command.angular.z)
        self._hal_proxy.SetSpeed(command.linear.x, command.angular.z)
        #rospy.logwarn("Twist command: {}".format(str(command)))

    def Start(self):
        pass

    def Loop(self):
        while not rospy.is_shutdown():

            odometry = self._hal_proxy.GetOdometry()
            rospy.logwarn("x: {:6.3f}, y: {:6.3f}, h: {:6.3f}, l: {:6.3f}, a: {:6.3f}".format(odometry['x_dist'], odometry['y_dist'], odometry['heading'], odometry['linear'], odometry['angular']))

            # adjust linear and angular speed by calibration factor
            #odometry['linear'] = odometry['linear'] * self._odom_linear_scale_correction
            #odometry['angular'] = odometry['angular'] * self._odom_angular_scale_correction

            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster,
                                           odometry['heading'], odometry['x_dist'], odometry['y_dist'], odometry['linear'], odometry['angular'])

            self._loop_rate.sleep()

        else:
            rospy.logwarn("Arlobot Drive Node: shutdown invoked, exiting")

    def Shutdown(self):
        rospy.loginfo("Arlobot Drive Node has shutdown")
        self._hal_proxy.SetSpeed(0, 0)


if __name__ == "__main__":
    arlobotdrivenode = ArlobotDriveNode()
    arlobotdrivenode.Start()
    arlobotdrivenode.Loop()
