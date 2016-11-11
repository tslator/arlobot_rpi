#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_odom_pub import ArlobotOdometryPublisher
from hal.base_hal_proxy import BaseHALProxy, BaseHALProxyError
from transform_utils import uni2diff, diff2uni

import time

class PID:
    def __init__(self, kp, ki, kd, min, max, sample_rate):
        self._target = 0
        self._output = 0
        self._error_sum = 0
        self._last_error = 0
        self._min = min
        self._max = max

        sample_time = 1.0/sample_rate

        self._kp = kp * sample_time
        self._ki = ki
        self._kd = kd / sample_time

        self._sign = 1.0

    def SetTarget(self, target):
        """

        :param target:
        :return:
        """

        '''
        One of the things that is problematic with PIDs and velocities is that the velocity can
        be positive and negative.  I have worked around this in other cases by always making
        the input to the PID a positive value and then keeping track of sign and adjusting the
        result.

        I would like to hide this 'adjustment' within the PID itself.
        '''
        self._sign = 1.0
        if target < 0.0:
            self._sign = -1.0
        self._target = abs(target)

    def Update(self, measured):
        """
        read e;
        e_dot = e - e_old;
        E = E + e;
        u = kp*e + kd*e_dot + ki*E;

        e_old = e;

        :param measured:
        :return:
        """

        # Calculate Error:
        error = self._target - measured

        e_dot = error - self._last_error
        self._error_sum += error
        self._output = self._kp*error + self._kd*e_dot + self._ki*self._error_sum

        self._last_error = error
        rospy.loginfo("min: {}, output: {}, max: {}".format(self._min, self._output, self._max))
        return min(max(self._output, self._min), self._max)*self._sign


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

        max_motor_rpm = rospy.get_param("Max Motor RPM", 95)
        wheel_diameter = rospy.get_param("Wheel Diameter", 0.1524)
        self._track_width = rospy.get_param("Track Width", 0.4030)

        # Calculate the wheel radius
        self._wheel_radius = wheel_diameter / 2.0

        # Calculate max wheel angular velocity in rad/s from RPM, i.e., 2*pi*RPMmax/60
        self._max_wheel_angular_velocity = 2 * math.pi * max_motor_rpm / 60

        # Calculate max linear and angular velocities
        v0, self._max_angular_velocity = diff2uni(self._max_wheel_angular_velocity,
                                                  -self._max_wheel_angular_velocity,
                                                  self._track_width,
                                                  self._wheel_radius)
        self._max_linear_velocity, w0 = diff2uni(self._max_wheel_angular_velocity,
                                                 self._max_wheel_angular_velocity,
                                                 self._track_width,
                                                 self._wheel_radius)

        max_linear_speed = rospy.get_param("Max Linear Speed", 1.0)
        max_angular_speed = rospy.get_param("Max Angular Speed", 1.0)

        self._max_linear_velocity = max(self._max_linear_velocity, max_linear_speed)
        self._max_angular_velocity = max(self._max_angular_velocity, max_angular_speed)

        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()

        loop_rate = rospy.get_param("Drive Node Loop Rate", 10)
        self._max_safety_timeout = rospy.get_param("Max Safety Timeout", 2.0)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDriveNodeError("Unable to create BaseHALProxy")

        self._hal_proxy.SetSpeed(0, 0)

        self._loop_rate = rospy.Rate(loop_rate)
        self._last_twist_time = time.time()

        self._last_linear_velocity = 0
        self._last_angular_velocity = 0
        self._linear_accel = rospy.get_param("Drive Node Velocity Profile Linear Accel", 3)
        self._angular_accel = rospy.get_param("Drive Node Velocity Profile Angular Accel", 0.1)
        self._linear_accel = 10
        self._angular_accel = 1

        self._theta = 0
        self._last_left_dist = 0
        self._last_right_dist = 0
        self._min_linear_velocity = 0
        self._min_angular_velocity = 0

        self._last_odom_time = time.time()
        self._left = 0.0
        self._right = 0.0

        kp = rospy.get_param("Linear Tracking Kp", 0.8)
        ki = rospy.get_param("Linear Tracking Ki", 0.0)
        kd = rospy.get_param("Linear Tracking Kd", 0.0)

        self._linear_tracking = PID(kp, ki, kd, self._min_linear_velocity, self._max_linear_velocity, loop_rate)

        kp = rospy.get_param("Angular Tracking Kp", 0.8)
        ki = rospy.get_param("Angular Tracking Ki", 0.0)
        kd = rospy.get_param("Angular Tracking Kd", 0.0)

        self._angular_tracking = PID(kp, ki, kd, self._min_angular_velocity, self._max_angular_velocity, loop_rate)

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def ensure_w(self, v, w):
        """

        :param v: linear velocity
        :param w: angular velocity
        :return:
        """
        '''
        1. Constrain the linear and angular velocities to robot minimum and maximum
           This prevents ROS from giving something that the robot motors can't achieve.
        '''
        w = max(min(w, self._max_angular_velocity), self._min_angular_velocity)
        v = max(min(v, self._max_linear_velocity), self._min_linear_velocity)

        '''
        2. Ensure that the given angular velocity is achieved by checking and adjusting
           the resulting differential velocities.
        '''
        l_v_d, r_v_d = uni2diff(v, w, self._track_width, self._wheel_radius)

        max_rl_v = max(l_v_d, r_v_d)
        min_rl_v = min(l_v_d, r_v_d)

        l_v = 0
        r_v = 0

        '''
        3. Adjust resulting left/right velocities to achieve desired angular velocity.
        '''
        if max_rl_v > self._max_wheel_angular_velocity:
            r_v = r_v_d - (max_rl_v - self._max_wheel_angular_velocity)
            l_v = l_v_d - (max_rl_v - self._max_wheel_angular_velocity)
        elif min_rl_v < -self._max_wheel_angular_velocity:
            r_v = r_v_d - (min_rl_v + self._max_wheel_angular_velocity)
            l_v = l_v_d - (min_rl_v + self._max_wheel_angular_velocity)
        else:
            l_v = l_v_d
            r_v = r_v_d

        v, w = diff2uni(l_v, r_v, self._track_width, self._wheel_radius)

        return v,w

    def _twist_command_callback(self, command):
        """
        Within this callback it is necessary to enforce velocity limits specific to preventing motor stalling and
        ensuring the requested angular velocity
        :param command:
        :return: None
        """

        self._last_twist_time = time.time()

        # Adjust commanded linear/angular velocities to ensure compliance
        v, w = self.ensure_w(command.linear.x, command.angular.z)

        self._linear_tracking.SetTarget(v)
        self._angular_tracking.SetTarget(w)

        left, right = uni2diff(command.linear.x,
                               command.angular.z,
                               self._track_width,
                               self._wheel_radius)
        rospy.loginfo("twist_command_callback - linear: {:6.3f}, angular: {:6.3f}, left: {:6.3f}, right: {:6.3f}".format(v, w, left, right))

    def _calc_odometry(self):
        odometry = self._hal_proxy.GetOdometry()
        rospy.loginfo("ls: {:6.3f}, rs: {:6.3f}, ld: {:6.3f}, rd: {:6.3f} hd: {:6.3f}".format(*odometry))

        left_speed, right_speed, left_dist, right_dist, heading = odometry

        delta_time = self._last_odom_time - time.time()
        self._last_odom_time = time.time()

        # Compare the heading
        # Note: The imu can provide a heading as well.  It will be interesting to see how they compare
        # rospy.loginfo("get imu")
        # imu = self._hal_proxy.GetImu()
        # rospy.loginfo("psoc heading: {:6.3f}, imu heading: {:6.3f}".format(heading, imu['euler']['heading']))

        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        left_delta_dist = self._last_left_dist - left_dist
        right_delta_dist = self._last_right_dist - right_dist

        self._last_left_dist = left_dist
        self._last_right_dist = right_dist

        c_dist = (left_delta_dist + right_delta_dist) / 2
        x_dist = c_dist * math.cos(self._theta) * delta_time
        y_dist = c_dist * math.cos(self._theta) * delta_time

        v, w = diff2uni(left_speed, right_speed, self._track_width, self._wheel_radius)

        return {'heading': self._theta, 'x_dist': x_dist, 'y_dist': y_dist, 'linear': v, 'angular': w}

    def _safety_timeout_exceeded(self):
        return time.time() - self._last_twist_time > self._max_safety_timeout

    def Start(self):
        self._hal_proxy.SetSpeed(0.0, 0.0)
        _, _, self._last_left_dist, self._last_right_dist, _ = self._hal_proxy.GetOdometry()
        rospy.loginfo("lld: {}, lrd: {}".format(self._last_left_dist, self._last_right_dist))

    def Loop(self):
        while not rospy.is_shutdown():
        
            odometry = self._calc_odometry()

            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster,
                                           odometry['heading'],
                                           odometry['x_dist'],
                                           odometry['y_dist'],
                                           odometry['linear'],
                                           odometry['angular'])

            # Consider implementing tracking of the linear/angular velocity at this level to ensure that the robot
            # is moving at the correct heading
            linear = self._linear_tracking.Update(odometry['linear'])
            rospy.loginfo("linear - target: {}, measured: {}, new: {}".format(self._linear_tracking._target,
                                                                              odometry['linear'],
                                                                              linear))
            # If there a need to be concerned with keeping the angle within in +/- pi?  Remember this is a problem
            # with angle tracking, but is it also a problem with angular velocity tracking?  If so, does atan2 care
            # if the parameter passed is radians vs radians/s?  If not, than math.atan2(sin(angular, cos(angular))
            # should do the trick.  It would be nice to somehow add this to the PID as a constraint to be checked before
            # returning the result.
            angular = self._angular_tracking.Update(odometry['angular'])
            rospy.loginfo("angular - target: {}, measured: {}, new: {}".format(self._angular_tracking._target,
                                                                               odometry['angular'],
                                                                               angular))
            left, right = uni2diff(linear, angular, self._track_width, self._wheel_radius)
            if self._safety_timeout_exceeded():
                left = 0.0
                right = 0.0

            self._hal_proxy.SetSpeed(left, right)

            rospy.loginfo("tick, tock {}".format(str(rospy.Time.now())))
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
