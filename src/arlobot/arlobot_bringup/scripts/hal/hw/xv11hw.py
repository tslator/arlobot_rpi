

class Xv11HwError(Exception):
    pass


class Xv11Hw:

    PORT = "/dev/ttyUSB0"
    BAUD = 115200


    def __init__(self, port, baud):
        self._port = port
        self._baud = baud

        # Need to connect to a serial gateway server with a worker thread and register a callback function
        self._ranges = []
        self._intensities = []
        self._time_increment = 0

    def _callback(self, data):
        """
        Read data from serial port and parse out ranges, intensities, and time_increment from the data
        :param data:
        :return:
        """

        # Add a lock so that accesses are serialized
        self._ranges = 0
        self._intensities = 0
        self._time_increment = 0


    def GetLaserScan(self):
        """
        """

        # Add a lock so that accesses are serialized
        return self._ranges, self._intensities, self._time_increment