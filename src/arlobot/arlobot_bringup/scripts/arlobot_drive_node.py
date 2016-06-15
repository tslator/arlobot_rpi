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
            raise BaseHALProxyError("Unable to create BaseHALProxy")

        self._hal_proxy.SetSpeed(0, 0)

        self._loop_rate = rospy.Rate(loop_rate)
        self._safety_delta_time = rospy.Time.now()
        self._last_twist_time = rospy.Time.now()

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def _apply_motion_profile(self):
        '''
        What's envisioned here is the ability to apply a motion profile, e.g., trapezoidal, S-curve, etc., to ensure
        smooth motor motion.  However, this may already be handled inherently by ROS in how it sends twist messages, so
        its not clear if it is necessary or how this could be applied.
        :return:
        '''
        pass

    def _twist_command_callback(self, command):
        self._safety_delta_time = rospy.Time.now() - self._last_twist_time
        self._last_twist_time = rospy.Time.now()

        self._apply_motion_profile()
        self._hal_proxy.SetSpeed(command.linear.x, command.angular.z)
        rospy.logwarn("Twist command: {}".format(str(command)))


    def Loop(self):
        while not rospy.is_shutdown():

            # Check that we are receiving Twist commands fast enough; otherwise, stop the motors
            if self._safety_delta_time.secs > self._safety_timeout:
                rospy.logdebug("Safety Timeout invoked: no twist command for {} seconds".format(self._safety_timeout))
                self._hal_proxy.SetSpeed(0, 0)

            odometry = self._hal_proxy.GetOdometry()
            #rospy.logwarn("x: {:6.3f}, y: {:6.3f}, h: {:6.3f}, l: {:6.3f}, a: {:6.3f}".format(odometry['x_dist'],
            #                                                                                  odometry['y_dist'],
            #                                                                                  odometry['heading'],
            #                                                                                  odometry['linear'],
            #                                                                                  odometry['angular']))

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
