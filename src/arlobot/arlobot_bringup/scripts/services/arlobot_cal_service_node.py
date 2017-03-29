#!/usr/bin/env python
from __future__ import print_function

"""
Description
"""

import rospy
from service_node import ServiceNode, ServiceNodeError
from utils.logging import rospylog
from arlobot_msgs.srv import CalCommand, CalCommandResponse


class ArlobotCalServiceNodeError(Exception):
    pass


class ArlobotCalServiceNode(ServiceNode):
    def __init__(self):
        ServiceNode.__init__(self, 'arlobot_cal_service_node', 'ArlobotCal')

    def _startup(self, timeout):
        rospy.loginfo("Starting up {} ...".format(self._service_name))
        rospy.loginfo("Successfully started {}".format(self._service_name))

        return True

    def _run(self):
        rospylog('info', "Creating services for {}".format(self._service_name))
        self._services['ArlobotCalMotor'] = rospy.Service('ArlobotCalMotor', 'CalMotor', CalCommand, self._cal_motor)
        self._services['ArlobotCalLeftPid'] = rospy.Service('ArlobotCalLeftPid', 'CalLeftPid', CalCommand, self._cal_left_pid)
        self._services['ArlobotCalRightPid'] = rospy.Service('ArlobotCalLeftPid', 'CalRightPid', CalCommand, self._cal_right_pid)
        self._services['ArlobotCalLinear'] = rospy.Service('ArlobotCalLeftPid', 'CalLinear', CalCommand, self._cal_linear)
        self._services['ArlobotCalAngular'] = rospy.Service('ArlobotCalLeftPid', 'CalAngular', CalCommand, self._cal_angular)
        rospylog('info', "{} Services Created".format(self._service_name))

    def _shutdown(self):
        rospy.loginfo("Shutting down {} ...".format(self._service_name))
        rospy.loginfo("Successfully shutdown {}".format(self._service_name))

        return True

    def _execute_command(self, command):
        """
        :param command:
        :return:
        """

        # Open the serial port server
        # Send command
        # Collect response lines
        srv = SerialServer(port="/dev/ttyAMA0", baud=115200)

        try:
            srv.send(command)
        except SerialServerError as e:
            rospylog('warn', "Failed to send command: {} - {}".format(command, e.args))

        try:
            response = srv.recv()
        except SerialServerError as e:
            rospylog('warn', "Failed to receive response: {}".format(e.args))

        return response

    def _dispatch(self, command, mode):
        handler = None
        response = ""
        if command in CalCommand.CAL_COMMANDS:
            handler = CalibrateHandler(serial_config, command, mode) as ch
            #with CalibrateHandler(serial_config, command, mode) as ch:
                response = ch.execute()
        elif command in CalCommand.DISP_COMMANDS:
            handler = DisplayHandler(command, mode)
        elif command in CalCommand.VAL_COMMANDS:
            handler = ValidateHandler(command, mode)
        else:
            # Unsupported command -> raise exception
            pass

        response = handler.execute()

        if mode == 'parsed':
            response = str(response)

        return response

    def _disp_motor(self, request):
        try:
            response = self._dispatch(request.command, request.mode)
        except ProtocolHandlerError as e:
            response = e.args

        return CalCommandResponse(response=response)

    def _cal_motor(self, request):
        """
        :description:
        From the request we know the 'command' (display, calibrate, validation), 'action' (motor, left/right pid, etc)
        and the 'mode' (raw or parsed).
        The routine also implies the protocol handler, e.g., _cal_motor vs _disp_motor, etc
        This information can be used to instantiate a protocol handler (DisplayHandler, CalibrateHandler, ValidateHandler)

        :param request:
        :return:
        """
        command = request.command
        mode = request.mode

        ch = CalibrateHandler(command, mode)

        #with CalibrateHandler() as ch:
        #    response = ch.motors()
        ch = CalibrateHandler(mode='parsed')
        response = ch.motors()

        return CalCommandResponse(response=str(response))


if __name__ == '__main__':
    nav_service = ArlobotCalServiceNode()
    nav_service.Start()
    nav_service.Run()


