#!/usr/bin/env python

import rospy
from ..hal_proxy import BaseHALProxy, BaseHALProxyError, PCHALProxy, PCHALProxyError
# Import some standard message for reporting power, battery etc


class PowerSensorError(Exception):
    pass


class PowerSensor:
    def __init__(self):
        """
         Power information is available from several sources on this system:
           - PC: there is a serial utility that talks to the power supply and report various power parameters.  This
             software would have to be integrated into a ROS node.
             Note: there should be a new hardware (hw) module for accessing this functionality.  Use the power supply
             model number (M4-ATX) to identify the module.
           - PowerPi: is a Raspberry Pi hat that can be powered by 12v and regulates 3.3v and 5v for the Raspberry Pi.
             Via the I2C bus, the PowerPi provides voltage and current measurements of the power supply, e.g., battery

         Given that the source of this data is from two different levels, one must decide where the participating ROS
         nodes live and how they communicate.

         Q: Can a node running on the PC access the HAL service exposed from the Raspberry Pi?  If so, then power monitoring
         might be better done from a node running on the PC which access receives power messages from ArlobotBaseNode or
         by directly accessing the HAL Proxy to retrieve that information.  Message vs Service call?

        """

        # Create publishers for messages
        self._base_power_publisher = rospy.Publisher("BasePower", BasePower, queue_size=10)
        self._pc_power_publisher = rospy.Publisher("PcPower", PcPower, queue_size=10)

        try:
            self._base_hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise PowerSensorError("Unable to create Base HAL")

        try:
            self._pc_hal_proxy = PCHALProxy()
        except PCHALProxyError:
            raise PowerSensorError("Unable to create PC HAL")



    def _base_power_msg(self, voltage, current, temperature):
        """
        Base supplies Voltage and Current values
        """
        msg = BasePower()
        msg.voltage = voltage
        msg.current = current
        msg.temp = temperature

        return msg

    def _pc_power_msg(self, params, temp):
        """
        PC supplies a number of power related values
        """
        msg = PcPower()
        msg.data = [params['VIN'], params['IGN'], params['33V'], params['5V'], params['12V'], temp]

        return msg

    def Publish(self):
        base_voltage = self._base_hal_proxy.GetVoltage()
        base_current = self._base_hal_proxy.GetCurrent()
        base_temperature = self._base_hal_proxy.GetTemp()
        self._base_power_publisher.publish(self._base_power_msg(base_voltage, base_current, base_temperature))

        params = self._pc_hal_proxy.GetPowerParams()
        temp = self._pc_hal_proxy.GetTemp()
        self._pc_power_publisher.publish(self._pc_power_msg(params, temp))
