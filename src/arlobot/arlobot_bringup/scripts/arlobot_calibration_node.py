#! /usr/bin/env python


"""
The arlobot calibration node provides a ROS interface to the motor calibration facility provided by the Psoc.

    * arlobot calibration node
        - runs on Arlobot
        - instantiates a calibration proxy
        - instantiates a message handled for calibration requests, e.g., perform motor calibration, perform pid calibration, etc
        - provides facility to displaying calibration data
            - Note: There is a PID visualization for ROS.  It would be nice to adapt that and use it for visualizing
              all of the calibration data

    * calibration proxy
        - runs on Arlobot
        - connects to the calibration service (running on the ArlobotBase)
        - marshals calibration requests to the calibration service
        - marshals calibration responses from the calibration service

    * calibration service
        - runs on Arlobotbase
        - instantiates communicating with the Psoc serial port
        - executes calibration requests
        - returns calibration responses

"""

import rospy


class ArlobotCalibrationNodeError(Exception):
    pass

class ArlobotCalibrationNode:

    NAME = 'arlobot_calibration_node'

    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('Arlobot Base Node Debug', False)
        if enable_debug:
            rospy.init_node(ArlobotCalibrationNode.NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(ArlobotCalibrationNode.NAME)
        rospy.on_shutdown(self.Shutdown)

        loop_rate_value = rospy.get_param("Base Node Loop Rate", 1)

        # Create a subscriber for calibration messages
        #   - DoMotorCalibration
        #   - DoLeftPidCalibration
        #   - DoRightPidCalibration
        #   - DoLinearCalibration
        #   - DoAngularCalibration
        #   - ReadMotorCalibration
        #   - ReadPidCalibration
        #   - ReadLinearCalibration
        #   - ReadAngularCalibration

    def Start(self):
        rospy.loginfo("Arlobot Calibration Node has started")

    def Loop(self):
        rospy.loginfo("Arlobot Calibration Node entering loop")
        while not rospy.is_shutdown():

            self._loop_rate.sleep()
        else:
            rospy.logwarn("Arlobot Base Node: shutdown invoked, exiting")

    def Shutdown(self):
        # Note: ROS does not guarantee that messages will be sent after shutdown, so we can't be sure that the status
        # message will be received.
        pass


if __name__ == "__main__":
    arlobotcalibrationnode = ArlobotCalibrationNode()
    arlobotcalibrationnode.Start()
    arlobotcalibrationnode.Loop()
