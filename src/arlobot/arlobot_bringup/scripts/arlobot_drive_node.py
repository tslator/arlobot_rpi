#!/usr/bin/env python

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from arlobot_exception import ArlobotError
from arlobot_odom_pub import ArlobotOdometryPublisher
from arlobot_diff_drive import ArlobotDifferentialDrive, ArlobotDifferentialDriveError
from msgs.msg import DriveSpeed, DriveStatus



class ArlobotDriveNodeError(ArlobotError):
    pass


class ArlobotDriveNode:
    def __init__(self):
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node('arlobot_drive_node', log_level=rospy.DEBUG)
        else:
            rospy.init_node('arlobot_drive_node')
        rospy.on_shutdown(self.Shutdown)

        self._OdometryTransformBroadcaster = tf.TransformBroadcaster()

        track_width = rospy.get_param("track_width", 0.4030)
        ticks_per_rev = rospy.get_param("tick_per_revoution", 72)
        wheel_diameter = rospy.get_param("wheel_diameter", 0.1524)
        max_linear_speed = rospy.get_param("max_linear_speed", 1.0)
        max_angular_speed = rospy.get_param("max_angular_speed", 1.0)
        gains = rospy.get_param("drive_node_gains", {"kp" : 1.0, "ki" : 0.1, "kd" : 0.01})
        loop_rate = rospy.get_param("drive_node_loop_rate", 5)
        diff_drive_loop_rate = rospy.get_param("diff_drive_loop_rate", 20)

        meter_per_tick = (math.pi * wheel_diameter) / ticks_per_rev
        drive_type = rospy.get_param("drive_node_drive_type", "ArlobotDifferential")

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