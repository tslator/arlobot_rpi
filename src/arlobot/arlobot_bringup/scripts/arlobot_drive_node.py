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
    def __init__(self, name, kp, ki, kd, min, max, sample_rate):
        self._name = name
        self._target = 0
        self._output = 0
        self._error_sum = 0
        self._last_error = 0
        self._min = abs(min)
        self._max = abs(max)

        sample_time = 1.0/sample_rate

        self._kp = kp * sample_time
        self._ki = ki
        self._kd = kd / sample_time

        self._sign = 1.0

    def SetTarget(self, target):
        """
	    :description: sets/updates the target value for the PID
        :param target: new target value
        :return:
        """

        # Restrict the target value to be positive between min and max
        # If the PID range is restricted to positive it becomes possible to generate a negative value.
        # In the case of wheels, this means they can go backwards which is undesirable for speed tracking
        # I'm sure there is better explanation in control system vernacular for this condition, but I
        # discovered this experimentally and found the easiest way to make things work is to keep them
        # positive and keep track of the sign so it can be applied to the final output
        self._sign = 1.0
        if target < 0.0:
            self._sign = -1.0
            self._target = abs(target)

    def Update(self, measured):
        """
        :description: basic PID algorithm which determines a new value based on measured input and applied control gains
            calc e;
            e_dot = e - e_old;
            E = E + e;
            u = kp*e + kd*e_dot + ki*E;
            e_old = e;

        :param measured:
        :return:
        """

        # Calculate error:
        error = self._target - measured

        # Calculate delta error
        e_dot = error - self._last_error
        # Calculate error integration
        self._error_sum += error

        # Calculate new output
        self._output = self._kp*error + self._kd*e_dot + self._ki*self._error_sum

        # Save error
        self._last_error = error

        rospy.loginfo("{} - min: {:6.3}, target: {:6.3}, output: {:6.3}, max: {:6.3}".format(self._name, float(self._min), float(self._target), float(self._output), float(self._max)))

        # Restrict the output to the min/max and apply the sign
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

        #--------------------------------
        # Wheel/Robot Velocity Configuration/Control
        #--------------------------------

        max_motor_rpm = rospy.get_param("Max Motor RPM", 95)
        wheel_diameter = rospy.get_param("Wheel Diameter", 0.1524)
        self._track_width = rospy.get_param("Track Width", 0.4030)

        # Calculate the wheel radius
        self._wheel_radius = wheel_diameter / 2.0

        # Calculate max wheel angular velocity in rad/s from RPM, i.e., 2*pi*RPMmax/60
        self._max_wheel_angular_velocity = 2 * math.pi * max_motor_rpm / 60

        # Calculate max linear and angular velocities based on motor parameters
        _, self._max_angular_velocity = diff2uni(self._max_wheel_angular_velocity,
                                                 -self._max_wheel_angular_velocity,
                                                 self._track_width,
                                                 self._wheel_radius)
        self._max_linear_velocity, _ = diff2uni(self._max_wheel_angular_velocity,
                                                self._max_wheel_angular_velocity,
                                                self._track_width,
                                                self._wheel_radius)

        # Get the 'user' linear/angular velocity settings
        max_linear_speed = rospy.get_param("Max Linear Speed", 1.0)
        max_angular_speed = rospy.get_param("Max Angular Speed", 0.3)

        # 'User' setting can be more restrictive but is limited to motor parameters
        self._min_linear_velocity = 0.0
        self._max_linear_velocity = min(self._max_linear_velocity, max_linear_speed)
        self._min_angular_velocity = 0.0
        self._max_angular_velocity = min(abs(self._max_angular_velocity), max_angular_speed)

        rospy.loginfo("---------------------------------------------------")
        rospy.loginfo("Max Linear Velocity : {:6.3}".format(self._max_linear_velocity))
        rospy.loginfo("Max Angular Velocity: {:6.3}".format(self._max_angular_velocity))
        rospy.loginfo("---------------------------------------------------")

        # Calculate range of linear velocities accepted from ROS
        self._max_forward_linear_velocity = max_linear_speed
        self._max_backward_linear_velocity = -max_linear_speed

        # Calculate range of angular velocities accepted from ROS
        self._max_ccw_angular_velocity = max_angular_speed
        self._max_cw_angular_velocity = -max_angular_speed

        self._linear_accel_limit = rospy.get_param("Drive Node Linear Accel Limit", 0.01)
        self._angular_accel_limit = rospy.get_param("Drive Node Angular Accel Limit", 0.002)

        self._linear = 0.0
        self._angular = 0.0

        #-----------------------------
        # Control/Processing
        #-----------------------------
        loop_rate = rospy.get_param("Drive Node Loop Rate", 10)
        self._max_safety_timeout = rospy.get_param("Drive Node Safety Timeout", 10.0)

        self._loop_rate = rospy.Rate(loop_rate)
        self._loop_time = 1.0/float(loop_rate)
        self._last_twist_time = time.time()

        #------------------------------------
        # Linear/Angular Tracking
        #------------------------------------

        # A layered appoach is used for control.  At the lowest level of control (within the Psoc) there are left/right PIDs
        # tuned to a as fast a response as possible.  However, that can still lead to differences between the wheels and
        # cause the robot to veer of course.  Here we have the next level of control where odometry is used to make adjustments
        # to the linear and angular velocity as received via ROS.  The PIDs here do not need to be as aggressive as in the Psoc.
        # They need only to ensures that any differences are corrected.  As can be seen, only the proportional gain is being
        # used for now.

        kp = rospy.get_param("Linear Tracking Kp", 0.7)
        ki = rospy.get_param("Linear Tracking Ki", 0.0)
        kd = rospy.get_param("Linear Tracking Kd", 0.0)

        self._linear_tracking = PID("linear", kp, ki, kd, self._min_linear_velocity, self._max_linear_velocity, loop_rate)

        kp = rospy.get_param("Angular Tracking Kp", 0.7)
        ki = rospy.get_param("Angular Tracking Ki", 0.0)
        kd = rospy.get_param("Angular Tracking Kd", 0.0)

        self._angular_tracking = PID("angular", kp, ki, kd, self._min_angular_velocity, self._max_angular_velocity, loop_rate)

        #------------------------------------
        # Odometry
        #------------------------------------
        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()

        self._last_left_dist = 0
        self._last_right_dist = 0
        self._min_linear_velocity = 0
        self._min_angular_velocity = 0

        self._last_odom_time = time.time()

        #-------------------------------------
        # Subscriptions
        #-------------------------------------

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        #-------------------------------------
        # Publications
        #-------------------------------------

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

        #---------------------------------------------------
        # Instantiate the HAL Base Proxy and stop the motors
        #---------------------------------------------------
        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ArlobotDriveNodeError("Unable to create BaseHALProxy")

        self._hal_proxy.SetSpeed(0, 0)


    def ensure_w(self, v, w):
        """
        :description: Ensure that the given angular velocity can be achieved.  Will reduce linear velocity as necessary
        :param v: linear velocity
        :param w: angular velocity
        :return:
        """

        '''
        Ensure that the given angular velocity is achieved by checking and adjusting the resulting differential velocities.
        '''
        l_v_d, r_v_d = uni2diff(v, w, self._track_width, self._wheel_radius)

        max_rl_v = max(l_v_d, r_v_d)
        min_rl_v = min(l_v_d, r_v_d)

        l_v = 0
        r_v = 0

        '''
        Adjust resulting left/right velocities to achieve desired angular velocity.
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
        :param command: the twist command received from the ROS stack
        :return: None
        """

        # Update time of twist command for safety purposes
        self._last_twist_time = time.time()

        # While there are subsequent checks linear and angular velocity, its seems prudent to apply the most course checking
        # here before allowing velocities to flow futher through the stack
        if command.linear.x < 0.0:
            v = min(command.linear.x, self._max_backward_linear_velocity)
        else:
            v = min(command.linear.x, self._max_forward_linear_velocity)

        if command.angular.z < 0.0:
            w = min(command.angular.z, self._max_cw_angular_velocity)
        else:
            w = min(command.angular.z, self._max_ccw_angular_velocity)

        rospy.loginfo("After b/f constrain - v: {:3}, w: {:3}".format(v, w))

        # Adjust commanded linear/angular velocities to ensure angular velocity (omega)
        v, w = self.ensure_w(command.linear.x, command.angular.z)

        rospy.loginfo("After Ensure - v: {:.3}, w: {:3}".format(v, w))

        # Update target velocities to be tracked
        self._linear_tracking.SetTarget(v)
        self._angular_tracking.SetTarget(w)

        # For debugging purposes, convert linear, angular and print current velocities request
        left, right = uni2diff(v, w, self._track_width, self._wheel_radius)
        rospy.loginfo("twist_command_callback - linear: {:6.3f}, angular: {:6.3f}, left: {:6.3f}, right: {:6.3f}".format(v, w, left, right))

    def _calc_odometry(self):
        """
        :description: Calculate the odometry parameters received from the robot and to be published to the ROS stack
        :return: None
        """

        # Get the latest odometry from the HAL
        odometry = self._hal_proxy.GetOdometry()
        #rospy.loginfo("ls: {:6.3f}, rs: {:6.3f}, ld: {:6.3f}, rd: {:6.3f} hd: {:6.3f}".format(*odometry))
        # Break out the odometry into its constituents
        # Note: heading is constrained to -Pi to Pi
        left_speed, right_speed, left_dist, right_dist, heading = odometry

        # Calculate the delta time to be used to calculate distance
        delta_time = self._last_odom_time - time.time()
        self._last_odom_time = time.time()

        # Compare the heading
        # Note: The imu can provide a heading as well.  It will be interesting to see how they compare
        # rospy.loginfo("get imu")
        # imu = self._hal_proxy.GetImu()
        # rospy.loginfo("psoc heading: {:6.3f}, imu heading: {:6.3f}".format(heading, imu['euler']['heading']))

        # Note: There is a problem reading from the IMU.  The IMU via Euler values provides a more accurate heading
        # Once the IMU is working, the heading should be taken from the Euler values.
        # Note: heading from Euler values is 0 .. 360.  Need to understand if that is acceptable for odometry
        # publishing and if not it will need to be adjusted, e.g., heading = euler_heading - 180 (?)
        

        left_delta_dist = self._last_left_dist - left_dist
        right_delta_dist = self._last_right_dist - right_dist

        self._last_left_dist = left_dist
        self._last_right_dist = right_dist

        c_dist = (left_delta_dist + right_delta_dist) / 2
        x_dist = c_dist * math.cos(heading) * delta_time
        y_dist = c_dist * math.cos(heading) * delta_time

        v, w = diff2uni(left_speed, right_speed, self._track_width, self._wheel_radius)

        return {'heading': heading, 'x_dist': x_dist, 'y_dist': y_dist, 'linear': v, 'angular': w}

    def _safety_timeout_exceeded(self):
        """
        :description: Checks if the time since the last twist command was received exceeds the specified safety timeout
        :return: True is the safety timeout has been exceeded; otherwise, False
        """
        return time.time() - self._last_twist_time > self._max_safety_timeout

    def _apply_accel_profile(self, linear, angular):
        """
        :description: Applies the accleration limit to the current velocity and cacluates a new velocity.
            Using this eq:
                a = (vi - vi-1)/t
                a will be fixed by selecting an acceleration
                t is fixed by the loop rate
                vi-1 is the last velocity
                vi is the new velocity
                vi = a*t + vi-1
        :param linear: the desired linear velocity
        :param angular: the desired angular velocity
        :return: None
        """

        # The purpose of this method is to limit the acceleration and prevent large and possibly damaging velocities
        # from be commanded.  This also can help reduce motor slippage and improve motion accuracy.  Coupled with
        # the tracking PIDs, limiting acceleration provides smooth transitions between motion commands.

        def calc_velocity(new, last, accel, delta):
            delta = new - last

            if delta < 0.0:
                velocity = max(-accel * delta + last, new)
            else:
                velocity = min(accel * delta + last, new)

            rospy.loginfo("v: {:6.3}, n: {:6.3}, l: {:6.3}, a: {:6.3}, d: {:6.3}".format(velocity, new, last, accel, delta))
            return velocity

        tolerance = 0.000001

        rospy.loginfo("lindelta: {:6.3}, angdelta: {:6.3}".format(self._linear - linear, self._angular - angular))
        if abs(self._linear - linear) > tolerance:
            self._linear = calc_velocity(linear, self._linear, self._linear_accel_limit, self._loop_time)
        else:
            self._linear = 0.0

        if abs(self._angular - angular) > tolerance:
            self._angular = calc_velocity(angular, self._angular, self._angular_accel_limit, self._loop_time)
        else:
            self._angular = 0.0

    def _track_commanded_velocity(self, odometry):
        """
        :description: Updates the linear/angular target velocity of the linear/angular PID controllers
        :param odometry: latest odometry calculation
        """

        linear = self._linear_tracking.Update(odometry['linear'])
        
        # If there a need to be concerned with keeping the angle within in +/- pi?  Remember this is a problem
        # with angle tracking, but is it also a problem with angular velocity tracking?  If so, does atan2 care
        # if the parameter passed is radians vs radians/s?  If not, than math.atan2(sin(angular, cos(angular))
        # should do the trick.  It would be nice to somehow add this to the PID as a constraint to be checked before
        # returning the result.
        angular = self._angular_tracking.Update(odometry['angular'])

        return linear, angular

    def _perform_safety_check(self):
        """
        :description: Performs various safety checks to ensure environment is safe for the robot
        :return: None
        """

        # Check the safety timeout.  Motion commands from ROS are latched so in order to keep from
        # running out of control, a check of the time since the last motion command is made.
        if self._safety_timeout_exceeded():
            self._linear = 0.0
            self._angular = 0.0

    def _update_speed(self):
        """
        :description: Convert the current linear/angular velocity to left/right speed and apply to the HAL
        :return: None
        """

        left, right = uni2diff(self._linear, self._angular, self._track_width, self._wheel_radius)
        rospy.loginfo("l: {:6.3}, a: {:6.3}, l: {:6.3}, r: {:6.3}".format(self._linear, self._angular, left, right))
        self._hal_proxy.SetSpeed(left, right)

    def Start(self):
        """
        :description: Start up the drive node
        :return: None
        """

        # There are likely other things that could be done in Start like performing safety checks and diagnostics
        # For now, we're just stopping the robot and initializing the last left/right distance
        self._hal_proxy.SetSpeed(0.0, 0.0)

        # Note: There is a command that can be sent to the HAL to clear the odometry.  It may make sense to do that here

        _, _, self._last_left_dist, self._last_right_dist, _ = self._hal_proxy.GetOdometry()
        rospy.logdebug("lld: {}, lrd: {}".format(self._last_left_dist, self._last_right_dist))

    def Loop(self):
        """
        Description: Main loop of the drive node and will run until the node is shutdown.  Within the loop,
            * odometry is collected and published,
            * track the commanded linear and angular velocity,
            * perform safety check
        :return: None
        """
        while not rospy.is_shutdown():
        
            odometry = self._calc_odometry()

            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster,
                                           odometry['heading'],
                                           odometry['x_dist'],
                                           odometry['y_dist'],
                                           odometry['linear'],
                                           odometry['angular'])

            linear, angular = self._track_commanded_velocity(odometry)
            self._apply_accel_profile(linear, angular)
            self._perform_safety_check()
            self._update_speed()

            self._loop_rate.sleep()

        else:
            rospy.logwarn("Arlobot Drive Node: shutdown invoked, exiting")

    def Shutdown(self):
        """
        Description: Sets the left/right speed of the motors to 0.0 to ensure the robot stops.  Note, the robot has a
        safety timeout that will also ensure the robot stops.
        :return: None
        """
        rospy.loginfo("Arlobot Drive Node has shutdown")
        self._hal_proxy.SetSpeed(0, 0)


if __name__ == "__main__":
    """
    Description: Instantiates the arlobot drive node, starts the node, and enters the main loop
    """
    arlobotdrivenode = ArlobotDriveNode()
    arlobotdrivenode.Start()
    arlobotdrivenode.Loop()
