#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_odom_pub import ArlobotOdometryPublisher
from arlobot_diff_drive import ArlobotDifferentialDrive, ArlobotDifferentialDriveError



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

        track_width = rospy.get_param("Track Width")
        ticks_per_rev = rospy.get_param("Tick Per Revolution")
        wheel_diameter = rospy.get_param("Wheel Diameter")
        max_linear_speed = rospy.get_param("Max Linear Speed")
        max_angular_speed = rospy.get_param("Max Angular Speed")
        gains = rospy.get_param("Drive Node Gains")
        loop_rate = rospy.get_param("Drive Node Loop Rate")
        diff_drive_loop_rate = rospy.get_param("Diff Drive Loop Rate")
        self._odom_linear_scale_correction = rospy.get_param("odom_linear_scale_correction", 1.0)
        self._odom_angular_scale_correction = rospy.get_param("odom_angular_scale_correction", 1.0)

        meter_per_tick = (math.pi * wheel_diameter) / ticks_per_rev
        drive_type = rospy.get_param("Drive Node Drive Type", "ArlobotDifferential")

        if drive_type.lower() == "ArlobotDifferential".lower():
            try:
                self._drive = ArlobotDifferentialDrive(track_width=track_width,
                                                       dist_per_count=meter_per_tick,
                                                       max_linear_speed=max_linear_speed,
                                                       max_angular_speed=max_angular_speed,
                                                       drive_sample_rate=diff_drive_loop_rate,
                                                       pid_params=gains)
            except ArlobotDifferentialDriveError as err:
                raise ArlobotDriveNodeError(err)


        self._loop_rate = rospy.Rate(loop_rate)

        # Subscriptions

        # Subscribes to the Twist message received from ROS proper
        self._twist_subscriber = rospy.Subscriber("cmd_vel", Twist, self._twist_command_callback)

        # Publications

        # Publishes the Odometry message
        self._arlobot_odometry = ArlobotOdometryPublisher()

    def _apply_motion_profile(self):
        pass

    def _twist_command_callback(self, command):
        #rospy.logdebug("Twist command: {}".format(str(command)))
        self._apply_motion_profile()
        self._drive.SetSpeed(command.linear.x, command.angular.z)

    def Start(self):
        rospy.loginfo("Arlobot Drive Node has started")

        # Start the drive thread
        self._drive.SetSpeed(0.0, 0.0)
        self._drive.Start()

    def Loop(self):
        while not rospy.is_shutdown():

            x_dist, y_dist, heading, linear_speed, angular_speed = self._drive.GetOdometry()
            #rospy.logdebug("x: {:6.3f}, y: {:6.3f}, h: {:6.3f}, ls: {:6.3f}, as: {:6.3f}".format(x_dist,
            #                                                                                     y_dist,
            #                                                                                     heading,
            #                                                                                     linear_speed,
            #                                                                                     angular_speed))

            # adjust linear and angular speed by calibration factor
            linear_speed = linear_speed * self._odom_linear_scale_correction
            angular_speed = angular_speed * self._odom_angular_scale_correction

            self._arlobot_odometry.Publish(self._OdometryTransformBroadcaster, heading, x_dist, y_dist, linear_speed, angular_speed)

            self._loop_rate.sleep()

        else:
            rospy.logwarn("Arlobot Drive Node: shutdown invoked, exiting")

    def Shutdown(self):
        rospy.loginfo("Arlobot Drive Node has shutdown")
        self._drive.SetSpeed(0.0, 0.0)
        self._drive.Stop()


if __name__ == "__main__":
    arlobotdrivenode = ArlobotDriveNode()
    arlobotdrivenode.Start()
    arlobotdrivenode.Loop()
