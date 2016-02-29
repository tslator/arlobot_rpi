from __future__ import print_function

#import Adafruit_GPIO as GPIO
#import Adafruit_GPIO.FT232H as FT232H
#from Adafruit_GPIO.FT232H import FT232H


def use_FT232H():
    pass


class FT232H:
    OUT = 0
    IN = 0
    HIGH = 0
    LOW = 0

    def setup(self, pin, dir):
        pass

    def output(self, pin, value):
        pass

    def input(self, pin):
        return 0


class UsbToGpioHwError(Exception):
    pass


class UsbToGpioHw:
    '''
    This class wraps the Adafuit GPIO and FT232H libraries in a Borg-style implementation so that other services can
    access the USB-to-GPIO FT232H hardware module.  The USB-to-GPIO FT232H hardware module is connected to the PC and
    provides access to 12 GPIO pins and other serial protocols (I2C, SPI, RS-485, etc).  At this point, the intent is
    to use the hardware module to control the Activity board power via a relay, monitor the relay status, and also
    monitor the relay status of the left and right motor relays.  Note: those relays are controlled from the Activity
    board.
    '''

    '''
    The FT232H supports 12 pins of GPIO: D4 to D7 and C0 to C7
    Pins numbers are mapped as follows:
        0 - 7 -> D0 - D7
        8 - 15 -> C0 - C7
    '''

    PIN_D0 = 0
    PIN_D1 = 1
    PIN_D2 = 2
    PIN_D3 = 3
    PIN_D4 = 4
    PIN_D5 = 5
    PIN_D6 = 6
    PIN_D7 = 7

    PIN_C0 = 8
    PIN_C1 = 9
    PIN_C2 = 10
    PIN_C3 = 11
    PIN_C4 = 12
    PIN_C5 = 13
    PIN_C6 = 14
    PIN_C7 = 15

    BOOL_TO_GPIO_MAP = {True : FT232H.HIGH, False: FT232H.LOW}
    GPIO_TO_BOOL_MAP = {FT232H.HIGH : True, FT232H.LOW : False}

    LOW_HIGH_LOW = 0
    HIGH_LOW_HIGH = 1

    def __init__(self):
        try:
            # Temporarily disable the built-in FTDI serial driver on Mac & Linux platforms.
            use_FT232H()
        except RuntimeError:
            raise UsbToGpioHwError("Unable to create FT232H")

        try:
            # Create an FT232H object that grabs the first available FT232H device found.
            self._ft232h = FT232H()
        except RuntimeError:
            raise UsbToGpioHwError("Unable to create FT232H")

    def SetOutput(self, pin, value):
        try:
            self._ft232h.setup(pin, FT232H.OUT)
            self._ft232h.output(pin, self.BOOL_TO_GPIO_MAP[value])
        except RuntimeError:
            raise UsbToGpioHwError("Error setting output")

    def GetInput(self, pin):
        result = False
        try:
            self._ft232h.setup(pin, FT232H.IN)
            result = self.GPIO_TO_BOOL_MAP[self._ft232h.input(pin)]
        except RuntimeError:
            raise UsbToGpioHwError("Error getting input")

        return result