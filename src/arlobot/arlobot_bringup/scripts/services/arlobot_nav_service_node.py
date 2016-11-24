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
from scripts.utils.logging import rospylog


class ArlobotNavServiceNodeError(Exception):
    pass


class ArlobotNavServiceNode(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self, 'arlobot_nav_service_node', 'ArlobotNav')

        # Setup for the Service
        self._cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
        self._base_frame = rospy.get_param('~base_frame', '/base_link')

        # The odom frame is usually just /odom
        self._odom_frame = rospy.get_param('~odom_frame', '/odom')

        self.__STOP_TWIST = self._make_twist(0.0, 0.0)

        # This is the rate at which we want to check for changes in odometry
        odom_rate = 50.0

        # Set the equivalent ROS rate variable
        self._odom_rate = rospy.Rate(odom_rate)
        self._odom_period = self._odom_rate.sleep_dur

        # This is the rate at which we want to issue velocity commands.
        # Note: Due to queuing effects, we don't want to issue a 'ton' of velocity messages as it
        # makes the system unresponsive.  The safety timeout is 2 seconds, so we only need to issue
        # commands less than 2 seconds or often enough to keep the robot moving -- presently set
        # to 2 Hz
        cmd_rate = 2.0
        self._cmd_duration = rospy.Duration(1.0/cmd_rate)

        # Initialize the tf listener
        self._tf_listener = tf.TransformListener()

        # Set the odom frame
        self._odom_frame = '/odom'

        # Linear state variables
        self._start_position = 0.0
        self._linear_goal = 0.0
        self._linear_tolerance = 0.0
        self._linear_disp = 0.0

        # Angular state variables
        self._start_angle = 0.0
        self._angular_goal = 0.0
        self._angular_tolerance = 0.0
        self._angular_disp = 0.0
        self._last_angle = 0.0

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self._tf_listener.waitForTransform(self._odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self._base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self._tf_listener.waitForTransform(self._odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self._base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def _startup(self, timeout):
        rospylog('info', "Starting up {} ...".format(self._service_name))
        rospylog('info', "Successfully started {}".format(self._service_name))

        return True

    def _run(self):
        rospylog('info', "Creating services for {}".format(self._service_name))
        self._services['ArlobotNavPosition'] = rospy.Service('ArlobotNavPosition', SetFloatArray, self._nav_position)
        self._services['ArlobotNavRotate'] = rospy.Service('ArlobotNavRotate', SetFloatArray, self._nav_rotate)
        self._services['ArlobotNavStraight'] = rospy.Service('ArlobotNavStraight', SetFloatArray, self._nav_straight)
        rospylog('info', "{} Services Created".format(self._service_name))

    def _shutdown(self):
        rospylog('info', "Shutting down {} ...".format(self._service_name))
        rospylog('info', "Successfully shutdown {}".format(self._service_name))

        return True

    def _get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self._tf_listener.lookupTransform(self._odom_frame, self._base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logerr("TF Exception")
            return

        position = Point(*trans)
        rotation = tf.transformations.euler_from_quaternion(rot)[2]
        rospylog('debug', "{}, {}".format(str(position), str(rotation)))

        return position, rotation

    def _at_linear_goal(self, position):
        # Compute the Euclidean distance from the start
        x2 = pow((position.x - self._start_position.x), 2)
        y2 = pow((position.y - self._start_position.y), 2)
        self._linear_disp = sqrt(x2 + y2)

        result = (self._linear_disp + self._linear_tolerance) > self._linear_goal
        rospylog('debug', "At Linear Goal: disp {:.3} goal {:.3}, ? {}".format(self._linear_disp,
                                                                               self._linear_goal,
                                                                               str(result)))
        return result

    def _update_turn_angle(self, rotation):
        delta_angle = rotation - self._last_angle
        normal_angle = atan2(sin(delta_angle), cos(delta_angle))
        self._angular_disp += abs(normal_angle)
        self._last_angle = rotation

    def _at_angular_goal(self, rotation):
        self._update_turn_angle(rotation)
        result = abs(self._angular_disp + self._angular_tolerance) >= abs(self._angular_goal)
        rospylog('debug', "At Angular Goal:  disp {:.3} goal {:.3}, ? {}".format(self._angular_disp,
                                                                                 self._angular_goal,
                                                                                 result))
        return result

    def _unpack_request(self, request, linear=True, angular=True):
        param_list = []
        
        if linear ^ angular:
            param_list += request.values[:3]
        elif linear and angular:              
            param_list += request.values[:-1]

        param_list.append(request.values[-1])

        return param_list

    def _init_timeout(self, timeout):
        self._timeout = rospy.Duration(timeout)
        self._last_time = rospy.Time.now()

    def _at_timeout(self):
        now = rospy.Time.now()
        delta_time = now - self._last_time
        self._last_time = now
        self._timeout = self._timeout - delta_time

        rospylog('debug', "Current timeout: {}".format(str(self._timeout)))

        return self._timeout <= rospy.Duration(0.0)

    def _init_nav_params(self):
        self._start_position = 0.0
        self._linear_goal = 0.0
        self._linear_tolerance = 0.0
        self._linear_disp = 0.0
        self._start_angle = 0.0
        self._angular_goal = 0.0
        self._angular_tolerance = 0.0
        self._angular_disp = 0.0
        self._last_angle = 0.0

    def _set_goal(self, linear_goal=0.0, linear_tolerance=0.001, angular_goal=0.0, angular_tolerance=0.001):
        position, rotation = self._get_odom()
        rospylog('debug', "{}, {}".format(str(position), str(rotation)))
        
        self._start_position = position
        self._linear_goal = linear_goal
        self._linear_tolerance = linear_tolerance
        self._linear_disp = 0.0

        self._start_angle = rotation
        self._angular_goal = angular_goal
        self._angular_tolerance = angular_tolerance
        self._angular_disp = 0.0
        self._last_angle = rotation

    def _at_goal(self, linear=True, angular=True):
        position, rotation = self._get_odom()
        rospylog('debug', "{}, {}".format(str(position), str(rotation)))
        at_linear_goal = True if not linear else self._at_linear_goal(position)
        at_angular_goal = True if not angular else self._at_angular_goal(rotation)
        at_timeout = self._at_timeout()
        rospylog('debug', "alg: {}, aag: {}, at: {}".format(str(at_linear_goal), str(at_angular_goal), str(at_timeout)))
        return at_linear_goal and at_angular_goal and not at_timeout

    def _make_twist(self, linear, angular):
        return Twist(Vector3(linear, 0.0, 0.0), Vector3(0.0, 0.0, angular))

    def _cmd_vel_stop(self):
        self._cmd_vel.publish(self.__STOP_TWIST)

    def _do_move(self, move, linear=True, angular=True):
        success = False
        cmd_timeout = self._cmd_duration
        self._cmd_vel.publish(move)
        while not self._at_goal(linear, angular):
            self._odom_rate.sleep()

            cmd_timeout -= self._odom_period
            if cmd_timeout <= rospy.Duration(0.0):
                self._cmd_vel.publish(move)
                cmd_timeout = self._cmd_duration

            if self._at_timeout():
                success = True
                break
        else:
            success = True

        rospylog('debug', "Issuing stop twist command")
        self._cmd_vel_stop()
        self._cmd_vel_stop()
        self._cmd_vel_stop()

        return success

    def _nav_position(self, request):
        goal_distance, linear_velocity, linear_tolerance, goal_angle, angular_velocity, angular_tolerance, timeout = self._unpack_request(request)

        linear_success = self._do_linear_move(goal_distance, linear_velocity, linear_tolerance, timeout)
        angular_success = self._do_angular_move(goal_angle, angular_velocity, angular_tolerance, timeout)

        return SetFloatArrayResponse(success=linear_success and angular_success)

    def _do_angular_move(self, angular_goal, angular_velocity, angular_tolerance, timeout):
        if angular_goal == 0.0:
            rospylog('debug', "No goal so we're just returning")
            self._cmd_vel_stop()
            return True

        self._init_nav_params()
        self._set_goal(angular_goal=angular_goal, angular_tolerance=angular_tolerance)
        self._init_timeout(timeout)

        move_cmd = self._make_twist(0.0, angular_velocity)

        return self._do_move(move_cmd, linear=False)

    def _do_linear_move(self, linear_goal, linear_velocity, linear_tolerance, timeout):
        if linear_goal == 0.0:
            rospylog('debug', "No goal so we're just returning")
            self._cmd_vel_stop()
            return True

        self._init_nav_params()
        self._set_goal(linear_goal=linear_goal, linear_tolerance=linear_tolerance)
        self._init_timeout(timeout)

        move_cmd = self._make_twist(linear_velocity, 0.0)

        return self._do_move(move_cmd, angular=False)

    def _nav_rotate(self, request):
        goal_angle, angular_velocity, angular_tolerance, timeout = self._unpack_request(request, linear=False)

        success = self._do_angular_move(goal_angle, angular_velocity, angular_tolerance, timeout)

        return SetFloatArrayResponse(success=success)

    def _nav_straight(self, request):
        goal_distance, linear_velocity, linear_tolerance, timeout = self._unpack_request(request, angular=False)

        success = self._do_linear_move(goal_distance, linear_velocity, linear_tolerance, timeout)

        return SetFloatArrayResponse(success=success)

if __name__ == '__main__':
    nav_service = ArlobotNavServiceNode()
    nav_service.Start()
    nav_service.Run()
