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

        self._last_linear = 0
        self._last_angular = 0
        self._linear_acceleration = rospy.get_param("Drive Node Velocity Profile Linear Accel", 3)
        self._angular_acceleration = rospy.get_param("Drive Node Velocity Profile Angular Accel", 0.1)
        self._time_slice = rospy.get_param("Drive Node Velocity Profile Time Slice", 0.01)
        velocity_profile_rate = rospy.get_param("Drive Node Velocity Profile Rate", 10)
        self._velocity_profile_max_time = rospy.get_param("Drive Node Velocity Profile Max Time", 0.08)
        self._velocity_profile_sample_time = 1.0/velocity_profile_rate

        if self._velocity_profile_max_time > self._velocity_profile_sample_time:
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
        pass

    def _twist_command_callback(self, command):
        self._hal_proxy.SetSpeed(command.linear.x, command.angular.z)
        #rospy.logwarn("Twist command: {}".format(str(command)))

    def Start(self):
        pass

    def Loop(self):
        while not rospy.is_shutdown():

            odometry = self._hal_proxy.GetOdometry()
            rospy.logwarn("x: {:6.3f}, y: {:6.3f}, h: {:6.3f}, l: {:6.3f}, a: {:6.3f}".format(odometry['x_dist'],
                                                                                              odometry['y_dist'],
                                                                                             odometry['heading'],
                                                                                            odometry['linear'],
                                                                                              odometry['angular']))

            # adjust linear and angular speed by calibration factor
            #odometry['linear'] = odometry['linear'] * self._odom_linear_scale_correction
            #odometry['angular'] = odometry['angular'] * self._odom_angular_scale_correction

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
        self._hal_proxy.SetSpeed(0, 0)


if __name__ == "__main__":
    arlobotdrivenode = ArlobotDriveNode()
    arlobotdrivenode.Start()
    arlobotdrivenode.Loop()
