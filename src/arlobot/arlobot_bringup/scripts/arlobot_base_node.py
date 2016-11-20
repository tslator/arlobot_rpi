#!/usr/bin/env python

# The Arlobot Base Node is the top level node of the Arlobotbase component.  The Arlobot Base Node is responsible for
# control and behavior that extends beyond a single node, e.g., safety, calibration, charging, etc.  The principle
# responsibility for the Arlobot Base Node is to startup the Arlobot Base HAL Service.

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Range
from arlobot_base_status_pub import ArlobotBaseStatusPublisher
from hal.base_hal_proxy import BaseHALProxy, BaseHALProxyError
from service_proxy import ServiceProxy, ServiceProxyError
from math import radians


class ArlobotBaseNodeError(Exception):
    pass


class ArlobotBaseNodeStates:
    STATE_UNKNOWN = "UNKNOWN"
    STATE_INITIAL = "INITIAL"
    STATE_STARTED = "STARTED"
    STATE_CALIBRATING = "CALIBRATING"
    STATE_RUNNING = "RUNNING"
    STATE_TIMEOUT = "TIMEOUT"
    STATE_SHUTDOWN = "SHUTDOWN"


class ArlobotBaseNode:

    NAME = 'arlobot_base_node'

    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('Arlobot Base Node Debug', False)
        if enable_debug:
            rospy.init_node(ArlobotBaseNode.NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(ArlobotBaseNode.NAME)
        rospy.on_shutdown(self.Shutdown)

        loop_rate_value = rospy.get_param("Base Node Loop Rate", 1)
        
        # Initialization
        self._operational_state =  ArlobotBaseNodeStates.STATE_INITIAL
        self._loop_rate = rospy.Rate(loop_rate_value)

        #--------------------------------------------
        # Startup the Service and Proxy
        #--------------------------------------------
        # Start the Arlobot Base Node service
        self._service = rospy.Service('ArlobotBaseStatus', Trigger, self._service_status)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError as e:
            raise ArlobotBaseNodeError("Unable to create BaseHALProxy: {}".format(e.args))


        # Subscriptions

        # Subscribes to battery status message received from ArlobotSafetyNode
        #rospy.Subscriber("battery_state", BatteryState, self._safety_callback)

        # Consider applying the skirt abstraction at this level.  We probably don't care about the manner in which
        # distance information is gathered, just that it is distance data that needs to be processed.  A skirt node
        # might be more appropriate then ultrasonic and infrared subscribers

        rospy.Subscriber("ultrasonic_array", Range, self._distance_callback)
        rospy.Subscriber("infrared_array", Range, self._distance_callback)

        # Publications

        # Publishes the Base Status message
        self._arlobot_base_status = ArlobotBaseStatusPublisher()

    def _update_state(self, state):
        if self._operational_state != state:
            rospy.loginfo("Publishing {} state".format(state))
        self._operational_state = state
        self._arlobot_base_status.Publish(self._operational_state)

    def _safety_callback(self, message):
        pass

    def _distance_callback(self, message):
        #rospy.loginfo("Distance: " + str(message))
        pass

    def _service_status(self, request):
        response = False

        while self._operational_state != ArlobotBaseNodeStates.STATE_RUNNING:
            rospy.loginfo("Waiting for Arlobot Base Node ...")
            rospy.sleep(0.5)
        else:
            response = True
            rospy.loginfo("Arlobot Base Node is running")
        return TriggerResponse(success=response, message="")

    def _perform_imu_cal_motion(self):
        """
        :description: This is yet to be determined, but likely the only imu component that is not calibrated is the
        magnetometer.  If that is the case, possibly rotating the robot +/-90 will cause the magnetometer to become
        calibrated.
        :return:
        """

        try:
            nav_proxy = ServiceProxy('NavService')
            result = nav_proxy.Rotate(radians(90), 0.5, 0.001)
            rospy.loginfo("Rotation Complete{}: {}, {}, {}".format(result, 90, 0.5, 0.001))
            nav_proxy.Rotate(radians(-180), 0.5, 0.001)
            rospy.loginfo("Rotation Complete{}: {}, {}, {}".format(result, -180, 0.5, 0.001))
            nav_proxy.Rotate(radians(90), 0.5, 0.001)
            rospy.loginfo("Rotation Complete{}: {}, {}, {}".format(result, 90, 0.5, 0.001))
        except ServiceProxyError as e:
            raise ArlobotBaseNodeError("Failed IMU motion: {}".format(e.args))

    def _perform_calibration_checks(self):
        """
        :description: Various components in the system need to be calibrated.  The policy will be to not let startup
        continue unless all components are calibrated.  Most calibration needs to be done on a component level.  The IMU
        magnetometer calibration depends on the location of the robot which in practice means that potentially it needs
        to be re-calibrated each time the robot starts up.  It would be helpful to find a sequence of basic and safe
        motions, e.g., rotating in place, that could be performed at Arlobot Base Node level.
        :return:
        """

        # The HAL provides access to calibration information for each component that needs it.  At this level, we care
        # about the overall calibration.  If everything is calibrated, then we're good; otherwise, we need to know if
        # the problem is just the IMU.  If the IMU is not calibrated

        if not self._hal_proxy.GetCalibrated():
            retries_remaining = 3
            while retries_remaining:
                detail = self._hal_proxy.GetCalibration()

                # Check to see what isn't calibrated.  Note, we can only fix magnetometer calibration
                if not detail['psoc']['calibrated']:
                    raise ArlobotBaseNodeError("Unable to proceed.  PSOC is not calibrated: {}".format(str(detail['psoc'])))

                imu_cal = detail['imu']['calibrated']
                imu_sys = detail['imu']['sys']
                imu_gyro = detail['imu']['gyro']
                imu_accel = detail['imu']['accel']
                imu_mag = detail['imu']['mag']

                # Only try to fix calibration if the imu is not calibrated and the gyro and accel are calibrated
                if not imu_cal and imu_gyro and imu_accel:
                    self._perform_imu_cal_motion()
                    retries_remaining -= 1
        else:
            return True

    def _perform_operational_checks(self):
        """
        :description: Not sure what an operational check is, but it seems like a reasonable think to have.  That makes
        no sense :-)
        :return:
        """
        pass

    def _perform_safety_checks(self):
        """
        :description: Check anything that could cause harm to the robot or things around the robot
        :return:
        """
        rospy.loginfo("Performing safety checks: ...")
        rospy.loginfo("\tDo we have enough charge?")
        rospy.loginfo("\tAre we too close to anything?")
        rospy.loginfo("\tAnything we should check?")

    def Start(self):
        rospy.loginfo("Arlobot Base Node has started")

        # Note: a little bit of sleep seems to be necessary to ensure that the INITIAL message is received.
        # Not sure why, but there is no harm in delaying the start of this node.
        # Q: Is there a way to poll rather than sleep?
        rospy.sleep(1)

        self._update_state(ArlobotBaseNodeStates.STATE_CHECKING)

        self._perform_calibration_checks()
        self._perform_operational_checks()

        self._update_state(ArlobotBaseNodeStates.STATE_STARTED)

    def Loop(self):
        rospy.loginfo("Arlobot Base Node entering loop")
        while not rospy.is_shutdown():
            self._update_state(ArlobotBaseNodeStates.STATE_RUNNING)

            # Consider implementing basic fundamental behaviors associated with robot safety and operation
            # at this level, i.e., obstacle avoidance, charging, etc
            # At this level (running on the raspberry pi) we have enough abstractions to implement behaviors and the
            # ability to switch between those behaviors as needed.
            #rospy.logdebug("tick, tock")

            self._perform_safety_checks()

            self._loop_rate.sleep()
        else:
            rospy.logwarn("Arlobot Base Node: shutdown invoked, exiting")
        
    def Shutdown(self):
        # Note: ROS does not guarantee that messages will be sent after shutdown, so we can't be sure that the status
        # message will be received.  However, services are guaranteed to complete before shutdown completes, so the
        # because there is a service call after this message is sent, the message should be received.
        # We're going to try anyway :-)
        self._update_state(ArlobotBaseNodeStates.STATE_SHUTDOWN)


if __name__ == "__main__":
    arlobotbasenode = ArlobotBaseNode()
    arlobotbasenode.Start()
    arlobotbasenode.Loop()

