import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class ArlobotOdometryPublisher:
    def __init__(self):
        self._publisher = rospy.Publisher("odom", Odometry, queue_size=10)

    def _msg(self, orientation, heading, x_dist, y_dist, linear_speed, angular_speed, now, use_pose_ekf=False):
        # next, we will publish the odometry message over ROS
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.header.stamp = now
        msg.pose.pose.position.x = x_dist
        msg.pose.pose.position.y = y_dist
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation = orientation

        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = linear_speed
        msg.twist.twist.linear.y = 0
        msg.twist.twist.angular.z = angular_speed

        if use_pose_ekf:
            # robot_pose_ekf needs these covariances and we may need to adjust them.
            # From: ~/turtlebot/src/turtlebot_create/create_node/src/create_node/covariances.py
            # However, this is not needed because we are not using robot_pose_ekf
            # odometry.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
            # 0, 1e-3, 0, 0, 0, 0,
            #                         0, 0, 1e6, 0, 0, 0,
            #                         0, 0, 0, 1e6, 0, 0,
            #                         0, 0, 0, 0, 1e6, 0,
            #                         0, 0, 0, 0, 0, 1e3]
            #
            # odometry.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
            #                          0, 1e-3, 0, 0, 0, 0,
            #                          0, 0, 1e6, 0, 0, 0,
            #                          0, 0, 0, 1e6, 0, 0,
            #                          0, 0, 0, 0, 1e6, 0,
            #                          0, 0, 0, 0, 0, 1e3]
            pass

        return msg

    def Publish(self, broadcaster, heading, x_dist, y_dist, linear_speed, angular_speed, use_pose_ekf=False):
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = math.sin(heading / 2.0)
        orientation.w = math.cos(heading / 2.0)

        ros_now = rospy.Time.now()

        # Publish the transform from frame odom to frame base_link over tf

        #    Note: pose ekf is how turtlebot does its thing.  If there is an imu and/or gyro, it might be best to take
        #    this approach
        if use_pose_ekf:
            # This transform conflicts with transforms built into the Turtle stack
            # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            # This is done in/with the robot_pose_ekf because it can integrate IMU/gyro data
            # using an "extended Kalman filter"
            # REMOVE this "line" if you use robot_pose_ekf
            pass
        else:
            broadcaster.sendTransform(
                (x_dist, y_dist, 0),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                ros_now,
                "base_footprint",
                "odom"
            )

        self._publisher.publish(self._msg(orientation, heading, x_dist, y_dist, linear_speed, angular_speed, ros_now, use_pose_ekf))
