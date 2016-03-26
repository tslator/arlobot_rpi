#! /usr/bin/env python
from __future__ import print_function

# PyUSB requires:
#  - At least one of the supported libraries (libusb 1.0, libusb 0.1 or OpenUSB)
# Installation
#   Debian
#     - sudo apt-get install python libusb-1.0-0
#   Setup
#     - sudo python setup.py install
#   Pip
#     - sudo pip install pyusb --pre
import usb.core
import usb.util
import time
import sys
sys.path.append("..")
from utils import Worker



class M4DeviceError(Exception):
    pass


class M4Device:

    VENDOR = 0x04d8
    PRODUCT = 0xd001
    READ_ENDPOINT = 0x81
    WRITE_ENDPOINT = 0x01
    TIMEOUT = 3000
    DIAG_CMD_RESPONSE_LEN = 24
    '''
    static size_t m4TypeLengths[13] = {
     1, 1, 1, 1, 1, 1, 1, 2,
     1, 2, 2, 1, 1
    };

    static float m4TypeConversionsV1[13] = {
     0.1123, 0.076625, 0.0375, 0.0188136,
     1.0, 1.0, 1.0, 1.0,
     10.0, 10.0, 1.0, 1.0,
     1.0
    };


    static float m4TypeConversionsV2[13] = {
     0.1552, 0.1165, 0.0389, 0.0195,
     1.0, 1.0, 1.0, 1.0,
     10.0, 10.0, 1.0, 1.0,
     1.0
    };
    '''

    # Note: These conversions were taken from the above structures which were taken from the M4 ATX zip file provided
    # by Mini Box (http://www.mini-box.com/M4-ATX).  Once the major version is known, we can choose the appropriate
    # conversions and eliminate the double dictionary
    __CONVERSIONS = {'VIN' : {'1' : 0.1123,    '2' : 0.1552},
                   'IGN' : {'1' : 0.1123,    '2' : 0.1552},
                   '33V' : {'1' : 0.0188136, '2' : 0.0195},
                   '5V'  : {'1' : 0.0375,    '2' : 0.0389},
                   '12V' : {'1' : 0.076625,  '2' : 0.1165}}

    def __init__(self):

        self._dev = usb.core.find(idVendor=M4Device.VENDOR, idProduct=M4Device.PRODUCT)
        if self._dev is None:
            raise ValueError('Device not found')

        if self._dev is None:
            raise ValueError('Device not found')
        else:
            print ("M4 ATX device found")

        reattach = False
        if self._dev.is_kernel_driver_active(0):
            print("device kernel driver is active")
            reattach = True
            self._dev.detach_kernel_driver(0)
            print("device kernel driver detached")


    def _parse(self, data):
        """
        Byte 0 must equal 21 (length?)
        Byte 2 Input Voltage
        Byte 3 Ignition Voltage
        Byte 4 3.3 voltage
        Byte 5 5.0 voltage
        Byte 6 12.0 voltage (how is 12 different than Input Voltage?)
        Byte 12 Temperature

        Byte 23 contains the M4 version.  Upper nibble is the major, Lower nibble is the minor
            - There are different conversion depending on the major version
               - If major version is 1, then V1 conversion should be used
               - If major version >= 2, then V2 conversion should be used
               - If major version is 0, then this is an error
        :param data:
        :return:
        """

        try:
            major_revision = chr((data[23] >> 4) & 0x0F)
            minor_revision = chr(data[23] & 0x0F)
        except ValueError:
            raise M4DeviceError("Error converting version: Major {}, Minor {}".format(major_revision, minor_revision))

        if major_revision == '0':
            raise M4DeviceError("Invalid version")

        return {'VIN' : data[2] * M4Device.__CONVERSIONS['VIN'][major_revision],
                'IGN' : data[3] * M4Device.__CONVERSIONS['IGN'][major_revision],
                '33V' : data[4] * M4Device.__CONVERSIONS['33V'][major_revision],
                '5V'  : data[5] * M4Device.__CONVERSIONS['5V'][major_revision],
                '12V' : data[6] * M4Device.__CONVERSIONS['12V'][major_revision],
                'Temp' : data[12]
               }

    def get_diag(self):
        """
        Read the diagnostic values and parse them into a dictionary
        :return:
        """
        num_bytes = self._dev.interruptWrite(M4Device.WRITE_ENDPOINT, [0x81, 0x00])
        if num_bytes != 2:
            return None

        data = self._dev.interruptRead(M4Device.READ_ENDPOINT, M4Device.DIAG_CMD_RESPONSE_LEN)
        print(data)
        if len(data) != 24:
            return None
        return self._parse(data)


class M4AtxHwError(Exception):
    pass


class M4AtxHw:


    def __init__(self):
        self._temp = 0
        self._voltage_in = 0
        self._voltage_ignition = 0
        self._33v = 0
        self._5v = 0
        self._12v = 0
        self._voltages = {'VIN' : self._voltage_in, 'IGN' : self._voltage_ignition, '33V' : self._33v, '5V' : self._5v, '12V' : self._12v}

        try:
            self._m4 = M4Device()
        except M4DeviceError:
            raise M4AtxHwError("Unable to create M4 device")

        self._worker = Worker("m4 xtv", self._read_data)

        print("working")

    def Start(self):
        print("Starting M4 ATX ...")
        self._worker.start()
        print("M4 ATX started")

    def Stop(self):
        print("Stopping M4 ATX ...")
        self._worker.stop()
        print("M4 ATX stopped")

    def _read_data(self):
        result = self._m4.get_diag()
        if result:
            self._voltages = result[:5]
            self._temp = result[5]
        time.sleep(1)

    def GetVoltages(self):
        return self._voltages

    def GetTemp(self):
        return {'c': self._temp}


if __name__ == "__main__":
    dev = M4Device()
    print(dev.get_diag())

    '''

    m4 = M4AtxHw()

    m4.Start()
    print(m4.GetVoltage())
    print(m4.GetTemp())
    m4.Stop()
    '''
