# The purpose of this node is to port the kinect auxillary node (https://github.com/muhrix/kinect_aux) from C++ to Python
#
# Additional information can be found here: http://fivedots.coe.psu.ac.th/~ad/jg/nui16/motorControl.pdf


import usb.core
import usb.util


class KinectAuxHwError(Exception):
    pass


class KinectAuxHw:
    VENDOR = 0x45e
    PRODUCT = 0x02b0

    GRAVITY = 9.80665
    FREENECT_COUNTS_PER_G = 819.0

    MIN_TILT_ANGLE = -31.0
    MID_TILT_ANGLE = 0.0
    MAX_TILT_ANGLE = 31.0

    MIN_LED = 0
    MAX_LED = 6
    LED_OFF = MIN_LED
    LED_GREEN = 1
    LED_RED = 2
    LED_ORANGE = 3
    LED_GREEN_BLINK = 4
    LED_ORANGE_BLINK = 5
    LED_RED_ORANGE_BLINK = MAX_LED

    MOTOR_STATUS_UNKNOWN = -1
    MOTOR_STATUS_STOPPED = 0
    MOTOR_STATUS_AT_LIMIT = 1
    MOTOR_STATUS_MOVING = 4
    MOTOR_STATUS_QUICK_BREAK = 8

    ACCEL_TILT_REQUEST_TYPE = 0xC0
    ACCEL_TILT_REQUEST = 0x32
    ACCEL_TILT_VALUE = 0x0
    ACCEL_TILT_INDEX = 0x0

    TILT_ANGLE_REQUEST_TYPE = 0x40
    TILT_ANGLE_REQUEST = 0x31
    TILT_ANGLE_INDEX = 0

    LED_REQUEST_TYPE = 0x40
    LED_REQUEST = 0x06
    LED_INDEX = 0

    def __init__(self):
        self._dev = usb.core.find(idVendor=KinectAuxHw.VENDOR, idProduct=KinectAuxHw.PRODUCT)
        if self._dev is None:
            raise ValueError('Device not found')

        usb.util.claim_interface(self._dev, 0)

    def _read_motor_state(self):
        buffer = self._dev.ctrl_transfer(KinectAuxHw.ACCEL_TILT_REQUEST_TYPE,
                                         KinectAuxHw.ACCEL_TILT_REQUEST,
                                         KinectAuxHw.ACCEL_TILT_VALUE,
                                         KinectAuxHw.ACCEL_TILT_INDEX,
                                         10)

        motor_state = {'speed' : int(buffer[1]),
                       'accel' : {
                                 'x' : int(buffer[2] << 8) | buffer[3],
                                 'y' : int(buffer[4] << 8) | buffer[5],
                                 'z' : int(buffer[6] << 8) | buffer[7]
                                 },
                       'tilt'  : {
                                 'angle'  : buffer[8],
                                 'status' : buffer[9]
                                 }
                      }

        return motor_state

    def GetSpeed(self):
        return self._read_motor_state()['speed']

    def GetAccel(self):
        return self._read_motor_state()['accel']

    def GetTilt(self):
        return self._read_motor_state()['tilt']

    def SetTilt(self, angle):
        angle = max(min(angle, KinectAuxHw.MAX_TILT_ANGLE), KinectAuxHw.MIN_TILT_ANGLE)
        angle = int(angle * 2)
        result = self._dev.ctrl_transfer(KinectAuxHw.TILT_ANGLE_REQUEST_TYPE,
                                         KinectAuxHw.TILT_ANGLE_REQUEST,
                                         angle,
                                         0,
                                         1)
        if result <= 0:
            msg = "Tilt Angle control transfer returned: {}".format(result)
            raise KinectAuxHwError(msg)

        # Wait for the movement to complete
        tilt = self._read_motor_state()['tilt']
        while tilt['status'] == KinectAuxHw.MOTOR_STATUS_MOVING:
            tilt = self._read_motor_state()['tilt']

    def SetLed(self, value):
        value = max(min(value, KinectAuxHw.MAX_LED), KinectAuxHw.MIN_LED)
        result = self._dev.ctrl_transfer(KinectAuxHw.LED_REQUEST_TYPE,
                                         KinectAuxHw.LED_REQUEST,
                                         int(value),
                                         0,
                                         1)
        if result <= 0:
            msg = "LED control transfer returned: {}".format(result)
            raise KinectAuxHwError(msg)


if __name__ == "__main__":
    import time
    ka = KinectAuxHw()
    print(ka.GetSpeed())
    print(ka.GetAccel())
    print(ka.GetTilt())
    ka.SetTilt(KinectAuxHw.MID_TILT_ANGLE)
    ka.SetTilt(KinectAuxHw.MAX_TILT_ANGLE)
    ka.SetTilt(KinectAuxHw.MID_TILT_ANGLE)
    ka.SetTilt(KinectAuxHw.MIN_TILT_ANGLE)
    ka.SetTilt(KinectAuxHw.MID_TILT_ANGLE)
    ka.SetTilt(0)
    ka.SetTilt(-3)
    ka.SetTilt(3)
    ka.SetTilt(0)
    for i in range(KinectAuxHw.MIN_LED, KinectAuxHw.MAX_LED + 1):
        print("Setting led value: ", i)
        ka.SetLed(i)
        time.sleep(3)
    ka.SetLed(0)










