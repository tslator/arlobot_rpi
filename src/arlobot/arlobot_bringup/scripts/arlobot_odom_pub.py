import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class ArlobotOdometryPublisher:
    def __init__(self):
        self._publisher = rospy.Publisher("odom", Odometry, queue_size=10)

    def _msg(self, quaternion, heading, x_dist, y_dist, linear_speed, angular_speed, now):
        # next, we will publish the odometry message over ROS
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.header.stamp = now
        msg.pose.pose.position.x = x_dist
        msg.pose.pose.position.y = y_dist
        msg.pose.pose.position.z = 0
        msg.pose.pose.orientation = quaternion

        msg.child_frame_id = "base_link"
        msg.twist.twist.linear.x = linear_speed
        msg.twist.twist.linear.y = 0
        msg.twist.twist.angular.z = angular_speed

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

        return msg

    def Publish(self, broadcaster, heading, x_dist, y_dist, linear_speed, angular_speed):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(heading / 2.0)
        quaternion.w = math.cos(heading / 2.0)

        ros_now = rospy.Time.now()

        # First, we'll publish the transform from frame odom to frame base_link over tf
        # Note that sendTransform requires that 'to' is passed in before 'from' while
        # the TransformListener' lookupTransform function expects 'from' first followed by 'to'.
        # This transform conflicts with transforms built into the Turtle stack
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        # This is done in/with the robot_pose_ekf because it can integrate IMU/gyro data
        # using an "extended Kalman filter"
        # REMOVE this "line" if you use robot_pose_ekf
        broadcaster.sendTransform(
            (x_dist, y_dist, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            ros_now,
            "base_footprint",
            "odom"
        )

        self._publisher.publish(self._msg(quaternion, heading, x_dist, y_dist, linear_speed, angular_speed, ros_now))
