#! /usr/bin/env python

import rospy
from arlobot_exception import ArlobotError


class DrivePidError(ArlobotError):
    pass


class DrivePid:
    def __init__(self, kp, ki, kd, input, output):
        self._kp = kp
        self._ki = ki
        self._kd = kd

        self._input = input
        self._output = output
        self._target = 0
        self._error = 0

        self.p_value = 0
        self._d_value = 0
        self._i_value = 0
        self._derivator = 0
        self._integrator = 0
        self._integrator_min = -0.5
        self._integrator_max = 0.5


    def SetTarget(self, target):
        self._target = target
        self._output(target)

    def Reset(self):
        pass

    def Update(self):
        """
        Calculate PID output value for given reference input and feedback
        """
        input = self._input()

        # error
        self._error = self._target - input
        if -0.001 > self._error > 0.001:
            self._error = 0.0

        self._p_value = self._kp * self._error
        self._d_value = self._kd * ( self._error - self._derivator )
        self._derivator = self._error

        self._integrator += self._error

        if not self._integrator_min > self._integrator > self._integrator_max:
            self._integrator = max(self._integrator_min, min(self._integrator, self._integrator_max))

        self._i_value = self._integrator * self._ki

        output = self._p_value + self._i_value + self._d_value

        #self._output(output)

        #rospy.loginfo("Target: {:6.2f}, Input: {:6.2f}, Error: {:6.2f}, Output: {:6.2f}".format(self._target, input, self._error, output))


