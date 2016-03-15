from __future__ import print_function

import time
import sys
import traceback
import serial
import math
import re
sys.path.append("../")
from utils import Worker

class Xv11HwError(Exception):
    pass


class Xv11Hw:

    #PORT = "/dev/ttyACM0"
    PORT = "COM7"
    BAUD = 115200

    def __init__(self, port, baud):
        self._port = port
        self._baud = baud

        # Need to connect to a serial gateway server with a worker thread and register a callback function
        self._ranges = []
        self._intensities = []
        self._time_increment = 0
        self._init_level = 0
        self._index = 0

        self._ranges = [0 for i in range(360)]
        self._intensities = [0 for i in range(360)]

        # Time Interval regular expression
        # Time Interval: <millis>\
        self._time_interval_re = re.compile(r'Time Interval: (\d+)')

        # RPM regular expression
        # RPM: <rpm>  PWM: <pwm>
        self._rpm_re = re.compile('RPM: (\d+\.\d+).*PWM:.*')

        # Distance regular expression
        # <angle>:  <dist> (<quality>)
        self._distance_re = re.compile('(\d+): (\d+) \((\d+)\)')

        try:
            self._ser = serial.Serial(self._port, self._baud)
        except serial.SerialException:
            raise Xv11HwError("Unable to instantiate serial port: {}".format(self._port))

        '''
        Note: The Surreal LIDAR supports ASCII text commands which can control the LIDAR, e.g., motor on/off and speed
        and the display of rpm, time, and scan information.

            - RelayOff - not sure what this command does, but it is given on Surreals web page
            - ShowRPM - adds RPM and PWM data to the serial stream
            - ShowDist - adds scan data, <angle>, <distance> and <quality> data to the serial stream
        '''

        # Set the LIDAR into command mode to output RPM, Time Interval and Angle/Distance/Quality to the serial port
        self._ser.write("RelayOff\n")

        self._ser.write("ShowRPM\n")
        # Responds with "Showing RPM data"
        self._ser.readline()
        self._ser.flushInput()

        self._ser.write("ShowDist\n")
        # Responds with "Showing Distance data"
        self._ser.readline()
        self._ser.flushInput()

        self._worker = Worker("xv11 serial", self._read_lidar)

    def _turn_on_motor(self):
        self._ser.write("MotorOn\n")
        self._ser.readline()
        self._ser.flushInput()

    def _turn_off_motor(self):
        self._ser.write("MotorOff\n")
        self._ser.readline()
        self._ser.flushInput()

    def _parse_rpm_line(self, line):
        result = self._rpm_re.match(line)
        if result:
            if result.group(1):
                self._speed_rpm = float(result.group(1))
                return True

        return False

    def _parse_time_interval_line(self, line):
        result = self._time_interval_re.match(line)
        if result:
            if result.group(1):
                # Note: XV11 reports time in milliseconds and scan time is in seconds
                self._scan_time = float(result.group(1))/1000
                return True

        return False

    def _parse_distance_line(self, line):
        result = self._distance_re.match(line)
        if result:
            if result.group(1) and result.group(2) and result.group(3):
                angle = int(result.group(1))
                if 0 < angle < 360:
                    dist = int(result.group(2))
                    quality = int(result.group(3))

                    self._ranges[angle] = dist
                    self._intensities[angle] = quality

                return True

        return False

    def _read_lidar(self):
        '''
        Read the serial data from the LIDAR.  It is expected that the LIDAR has been enabled to output:
            Distance data
            RPM data
            Time Interval data
        :return:
        '''

        line = self._ser.readline()
        if self._parse_rpm_line(line):
            pass
        elif self._parse_time_interval_line(line):
            pass
        elif self._parse_distance_line(line):
            pass
        else:
            print("No matching parse for this line: ", line)

    def Start(self):
        print("Starting xv11 ...")
        self._turn_on_motor()
        self._worker.start()
        print("xv11 is running")

    def Stop(self):
        print("Stopping xv11 ...")
        self._turn_off_motor()
        self._worker.stop()
        del(self._ser)
        print("xv11 is stopped")

    def GetLaserScan(self):
        """
        """
        # Consider adding a lock so that accesses are serialized
        return self._ranges, self._intensities, self._scan_time

    def Show(self):
        print(self._speed_rpm)
        print(self._scan_time)
        print(self._ranges)
        print(self._intensities)


if __name__ == "__main__":
    xv11 = Xv11Hw(Xv11Hw.PORT, Xv11Hw.BAUD)
    xv11.Start()
    for i in range(5):
        time.sleep(1)
        xv11.Show()
    xv11.Stop()
    xv11.Show()
    #ser = serial.Serial(Xv11Hw.PORT, Xv11Hw.BAUD)
    #ser.write("MotorOff\n")

