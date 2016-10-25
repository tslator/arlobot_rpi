#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_odom_pub import ArlobotOdometryPublisher
from hal.base_hal_proxy import BaseHALProxy, BaseHALProxyError

import time

class PID:
    def __init__(self, kp, ki, kd, min, max, sample_rate):
        self._target = 0
        self._output = 0
        self._error_sum = 0
        self._last_error = 0
        self._min = min
        self._max = max

        self._kp = kp * sample_rate
        self._kd = kd / sample_rate

    def SetTarget(self, target):
        self._target = target

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

        return min(max(self._output, self._min), self._max)


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
        v0, self._max_angular_velocity = self.diff2uni(self._max_wheel_angular_velocity, -self._max_wheel_angular_velocity)
        self._max_linear_velocity, w0 = self.diff2uni(self._max_wheel_angular_velocity, self._max_wheel_angular_velocity)

        max_linear_speed = rospy.get_param("Max Linear Speed", 1.0)
        max_angular_speed = rospy.get_param("Max Angular Speed", 1.0)

        self._max_linear_velocity = max(self._max_linear_velocity, max_linear_speed)
        self._max_angular_velocity = max(self._max_angular_velocity, max_angular_speed)

        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()

        loop_rate = rospy.get_param("Drive Node Loop Rate")

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

        self._theta = 0
        self._min_linear_velocity = 0
        self._min_angular_velocity = 0

        kp = rospy.get_param("Linear Tracking Kp", 1.0)
        ki = rospy.get_param("Linear Tracking Ki", 0.0)
        kd = rospy.get_param("Linear Tracking Kd", 0.0)

        self._linear_tracking = PID(kp, ki, kd, self._min_linear_velocity, self._max_linear_velocity, loop_rate)

        kp = rospy.get_param("Angular Tracking Kp", 1.0)
        ki = rospy.get_param("Angular Tracking Ki", 0.0)
        kd = rospy.get_param("Angular Tracking Kd", 0.0)

        self._angular_tracking = PID(kp, ki, kd, self._min_angular_velocity, self._max_angular_velocity, loop_rate)

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def _apply_accel_profile(self, new_linear, new_angular):
        """
        Calculate the max linear and angular velocity based on max linear and angular acceleration (configurable via parameter)
        Choose the min of new and max linear and angular velocity
        :param new_linear:
        :param new_angular:
        :return:
        """

        delta_time = self._last_twist_time - rospy.time.now() # needs to be in seconds

        max_linear_velocity = self._last_linear_velocity + self._max_linear_accel * delta_time
        max_angular_velocity = self._last_angular_velocity + self._max_angular_accel * delta_time
        self._last_linear_velocity = min(new_linear, max_linear_velocity)
        self._last_angular_velocity = min(new_angular, max_angular_velocity)

        return self._last_linear_velocity, self._last_angular_velocity

    def uni2diff(self, v, w):
        summ = 2*v/self._wheel_radius
        diff = self._track_width*w/self._wheel_radius

        l_v = (summ-diff)/2
        r_v = (summ+diff)/2

        return l_v, r_v

    def diff2uni(self, l_v, r_v):
        v = (l_v + r_v) * (self._wheel_radius / 2)
        w = (r_v - l_v) * (self._wheel_radius / self._track_width)

        return v, w

    def ensure_w(self, v, w):
        '''
        1. Constrain the linear and angular velocities to robot minimum and maximum
           This prevents ROS from giving something that the robot motors can't achieve
        '''
        w = max(min(w, self._max_angular_velocity), self._min_angular_velocity)
        v = max(min(v, self._max_linear_velocity), self._min_linear_velocity)

        '''
        2. Ensure that the given angular velocity is achieved by checking and adjusting the resulting differential velocities
        '''
        l_v_d, r_v_d = self.uni2diff(v, w)

        max_rl_v = max(l_v_d, r_v_d)
        min_rl_v = min(l_v_d, r_v_d)

        l_v = 0
        r_v = 0

        if max_rl_v > self._max_wheel_angular_velocity:
            r_v = r_v_d - (max_rl_v - self._max_wheel_angular_velocity)
            l_v = l_v_d - (max_rl_v - self._max_wheel_angular_velocity)
        elif min_rl_v < -self._max_wheel_angular_velocity:
            r_v = r_v_d - (min_rl_v + self._max_wheel_angular_velocity)
            l_v = l_v_d - (min_rl_v + self._max_wheel_angular_velocity)
        else:
            l_v = l_v_d
            r_v = r_v_d

        v, w = self.diff2uni(l_v, r_v)

    def _twist_command_callback(self, command):
        """
        Within this callback it is necessary to enforce velocity limits specific to preventing motor stalling and
        ensuring the requested angular velocity
        :param command:
        :return: None
        """

        # Adjust commanded linear/angular velocities to ensure compliance
        v, w = self.ensure_w(command.linear.x, command.angular.z)

        # Apply an acceleration profile to ensure smooth transitions
        v, w = self._apply_accel_profile(v, w)

        # Update the tracking PIDs with the latest values
        self._linear_tracking.SetTarget(v)

        self._angular_tracking.SetTarget(w)

    def Start(self):
        pass

    def CalculeOdometry(self):

        odometry = self._hal_proxy.GetOdometry()
        rospy.logwarn("ls: {:6.3f}, rs: {:6.3f}, ld: {:6.3f}, rd: {:6.3f} hd: {:6.3f}".format(*odometry))

        left_speed, right_speed, left_delta_dist, right_delta_dist, heading = odometry

        delta_time = self._last_odom_time - time.time()
        self._last_odom_time = time.time()

        # Compare the heading
        # Note: The imu can provide a heading as well.  It will be interesting to see how they compare
        imu = self._hal_proxy.GetImu()
        rospy.logwarn("psoc heading: {:6.3f}, imu heading: {:6.3f}".format(heading, imu['heading']))

        self._theta = math.atan2(math.sin(self._theta), math.cos(self._theta))

        c_dist = (left_delta_dist + right_delta_dist)/2
        x_dist = c_dist*math.cos(self._theta)*delta_time
        y_dist = c_dist*math.cos(self._theta)*delta_time

        v, w = self.diff2uni(left_speed, right_speed)

        return {'heading':self._theta, 'x_dist': x_dist, 'y_dist': y_dist, 'linear': v, 'angular': w}

    def Loop(self):
        while not rospy.is_shutdown():


            odometry = self._CalculateOdometry()
            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster,
                                           odometry['heading'],
                                           odometry['x_dist'],
                                           odometry['y_dist'],
                                           odometry['linear'],
                                           odometry['angular'])


            # Consider implementing tracking of the linear/angular velocity at this level to ensure that the robot
            # is moving at the correct heading
            linear = self._linear_tracking.Update(odometry['linear'])
            # If there a need to be concerned with keeping the angle within in +/- pi?  Remember this is a problem
            # with angle tracking, but is it also a problem with angular velocity tracking?  If so, does atan2 care
            # if the parameter passed is radians vs radians/s?  If not, than math.atan2(sin(angular, cos(angular))
            # should do the trick.  It would be nice to somehow add this to the PID as a constraint to be checked before
            # returning the result.
            angular = self._angular_tracking.Update(odometry['angular'])

            self._hal_proxy.SetSpeed(linear, angular)

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
