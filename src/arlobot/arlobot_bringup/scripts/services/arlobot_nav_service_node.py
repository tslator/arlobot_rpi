#!/usr/bin/env python

# The purpose of this service node is to consolidate a tf listener into a single generic node that can take motion
# parameters, goal distance, goal angle, linear and angular velocity, execute those parameters and provide notification
# when complete.  This sounds very similar to the simple action client/server, but is less involved and it is a
# generalization of more specific nodes like, 'out and back' and 'square'.  Both of these nodes duplicate code for tf
# processing.

# A simlar approach will be taken with this service as is done for the HAL services in that there will be a service and
# a proxy.  There will be one instance of the service and possibly multiple instances of the proxy.

import time
from math import sqrt, atan2, sin, cos
import rospy
import tf
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3
from arlobot_msgs.srv import SetFloatArray, SetFloatArrayResponse
from service_node import ServiceNode, ServiceNodeError


class ArlobotNavServiceNodeError(Exception):
    pass


class ArlobotNavServiceNode(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self, 'arlobot_nav_service_node', 'NavService')

        # Setup for the Service
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self.base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')

        rate = 20

        # Set the equivalent ROS rate variable
        self._rate = rospy.Rate(rate)


        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def _startup(self, timeout):
        rospy.loginfo("Starting up {} ...".format(self._service_name))
        rospy.loginfo("Successfully started {}".format(self._service_name))

        return True

    def _run(self):
        self._services['NavPosition'] = rospy.Service('NavPosition', SetFloatArray, self._nav_position)
        self._services['NavRotate'] = rospy.Service('NavRotate', SetFloatArray, self._nav_rotate)
        self._services['NavStraight'] = rospy.Service('NavStraight', SetFloatArray, self._nav_straight)

    def _shutdown(self):
        rospy.loginfo("Shutting down {} ...".format(self._service_name))
        rospy.loginfo("Successfully shutdown {}".format(self._service_name))

        return True

    def _get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        #return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        return Point(*trans), tf.transformations.euler_from_quaternion(Quaternion(*rot))[0]

    def _at_distance_goal(self, x_start, y_start, position, goal, tolerance):
        # Compute the Euclidean distance from the start
        distance = sqrt(pow((position.x - x_start), 2) + pow((position.y - y_start), 2))

        return distance > goal

    def _at_angle_goal(self, start_angle, rotation, goal, tolerance):
        #angle = normalize_angle(rotation - start_angle)
        delta_angle = rotation - start_angle
        angle = atan2(sin(delta_angle), cos(delta_angle))

        return abs(angle + tolerance) > abs(goal)

    def _unpack_request(self, request, linear=True, angular=True):
        param_list = []
        if linear:
            param_list += request[:3]
        if angular:
            param_list += request[3:-1]

        param_list.append(request[-1])

        return param_list

    def _nav_position(self, request):
        success = False

        goal_distance, linear_velocity, linear_tolerance, goal_angle, angular_velocity, angular_tolerance, timeout = self._unpack_request(request)

        move_cmd = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))
        (position, rotation) = self._get_odom()
        x_start = position.x
        y_start = position.y
        start_angle = rotation

        start_time = time.time()
        while not self._at_distance_goal(x_start, y_start, position, goal_distance, linear_tolerance) and \
              not self._at_angle_goal(start_angle, rotation, goal_angle, angular_tolerance) and not timeout:
            self.cmd_vel.publish(move_cmd)
            (position, rotation) = self._get_odom()
            timeout -= time.time() - start_time

            self._rate.sleep()
        else:
            success = True

        return SetFloatArrayResponse(success=success)

    def _nav_rotate(self, request):
        goal_angle, angular_velocity, angular_tolerance, timeout = self._unpack_request(request, linear=False)

        move_cmd = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, angular_velocity))
        (position, rotation) = self._get_odom()
        start_angle = rotation

        start_time = time.time()
        while not self._at_angle_goal(start_angle, rotation, goal_angle, angular_tolerance) and not timeout:
            self.cmd_vel.publish(move_cmd)
            (position, rotation) = self._get_odom()
            timeout -= time.time() - start_time

            self._rate.sleep()
        else:
            success = True

        return SetFloatArrayResponse(success=success)

    def _nav_straight(self, request):
        success = False
        goal_distance, linear_velocity, linear_tolerance, timeout = self._unpack_request(request, angular=False)

        move_cmd = Twist(Vector3(linear_velocity, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        (position, rotation) = self._get_odom()
        x_start = position.x
        y_start = position.y

        start_time = time.time()
        while not self._at_distance_goal(x_start, y_start, position, goal_distance, linear_tolerance) and not timeout:
            self.cmd_vel.publish(move_cmd)
            (position, rotation) = self._get_odom()
            timeout -= time.time() - start_time

            self._rate.sleep()
        else:
            success = True

        return SetFloatArrayResponse(success=success)


if __name__ == '__main__':
    nav_service = ArlobotNavServiceNode()
    nav_service.Start()
    nav_service.Run()
