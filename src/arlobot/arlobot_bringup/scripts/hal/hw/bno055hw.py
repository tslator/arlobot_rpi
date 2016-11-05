from __future__ import print_function

import time
from imuhw import HardwareError

# I2C addresses
BNO055_ADDRESS_A                     = 0x28
BNO055_ADDRESS_B                     = 0x29
BNO055_ID                            = 0xA0

# Page id register definition
BNO055_PAGE_ID_ADDR                  = 0X07

# PAGE0 REGISTER DEFINITION START
BNO055_CHIP_ID_ADDR                  = 0x00
BNO055_ACCEL_REV_ID_ADDR             = 0x01
BNO055_MAG_REV_ID_ADDR               = 0x02
BNO055_GYRO_REV_ID_ADDR              = 0x03
BNO055_SW_REV_ID_LSB_ADDR            = 0x04
BNO055_SW_REV_ID_MSB_ADDR            = 0x05
BNO055_BL_REV_ID_ADDR                = 0X06

# Accel data register
BNO055_ACCEL_DATA_X_LSB_ADDR         = 0X08
BNO055_ACCEL_DATA_X_MSB_ADDR         = 0X09
BNO055_ACCEL_DATA_Y_LSB_ADDR         = 0X0A
BNO055_ACCEL_DATA_Y_MSB_ADDR         = 0X0B
BNO055_ACCEL_DATA_Z_LSB_ADDR         = 0X0C
BNO055_ACCEL_DATA_Z_MSB_ADDR         = 0X0D

# Mag data register
BNO055_MAG_DATA_X_LSB_ADDR           = 0X0E
BNO055_MAG_DATA_X_MSB_ADDR           = 0X0F
BNO055_MAG_DATA_Y_LSB_ADDR           = 0X10
BNO055_MAG_DATA_Y_MSB_ADDR           = 0X11
BNO055_MAG_DATA_Z_LSB_ADDR           = 0X12
BNO055_MAG_DATA_Z_MSB_ADDR           = 0X13

# Gyro data registers
BNO055_GYRO_DATA_X_LSB_ADDR          = 0X14
BNO055_GYRO_DATA_X_MSB_ADDR          = 0X15
BNO055_GYRO_DATA_Y_LSB_ADDR          = 0X16
BNO055_GYRO_DATA_Y_MSB_ADDR          = 0X17
BNO055_GYRO_DATA_Z_LSB_ADDR          = 0X18
BNO055_GYRO_DATA_Z_MSB_ADDR          = 0X19

# Euler data registers
BNO055_EULER_H_LSB_ADDR              = 0X1A
BNO055_EULER_H_MSB_ADDR              = 0X1B
BNO055_EULER_R_LSB_ADDR              = 0X1C
BNO055_EULER_R_MSB_ADDR              = 0X1D
BNO055_EULER_P_LSB_ADDR              = 0X1E
BNO055_EULER_P_MSB_ADDR              = 0X1F

# Quaternion data registers
BNO055_QUATERNION_DATA_W_LSB_ADDR    = 0X20
BNO055_QUATERNION_DATA_W_MSB_ADDR    = 0X21
BNO055_QUATERNION_DATA_X_LSB_ADDR    = 0X22
BNO055_QUATERNION_DATA_X_MSB_ADDR    = 0X23
BNO055_QUATERNION_DATA_Y_LSB_ADDR    = 0X24
BNO055_QUATERNION_DATA_Y_MSB_ADDR    = 0X25
BNO055_QUATERNION_DATA_Z_LSB_ADDR    = 0X26
BNO055_QUATERNION_DATA_Z_MSB_ADDR    = 0X27

# Linear acceleration data registers
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  = 0X28
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR  = 0X29
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR  = 0X2A
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR  = 0X2B
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR  = 0X2C
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR  = 0X2D

# Gravity data registers
BNO055_GRAVITY_DATA_X_LSB_ADDR       = 0X2E
BNO055_GRAVITY_DATA_X_MSB_ADDR       = 0X2F
BNO055_GRAVITY_DATA_Y_LSB_ADDR       = 0X30
BNO055_GRAVITY_DATA_Y_MSB_ADDR       = 0X31
BNO055_GRAVITY_DATA_Z_LSB_ADDR       = 0X32
BNO055_GRAVITY_DATA_Z_MSB_ADDR       = 0X33

# Temperature data register
BNO055_TEMP_ADDR                     = 0X34

# Status registers
BNO055_CALIB_STAT_ADDR               = 0X35
BNO055_SELFTEST_RESULT_ADDR          = 0X36
BNO055_INTR_STAT_ADDR                = 0X37

BNO055_SYS_CLK_STAT_ADDR             = 0X38
BNO055_SYS_STAT_ADDR                 = 0X39
BNO055_SYS_ERR_ADDR                  = 0X3A

# Unit selection register
BNO055_UNIT_SEL_ADDR                 = 0X3B
BNO055_DATA_SELECT_ADDR              = 0X3C

# Mode registers
BNO055_OPR_MODE_ADDR                 = 0X3D
BNO055_PWR_MODE_ADDR                 = 0X3E

BNO055_SYS_TRIGGER_ADDR              = 0X3F
BNO055_TEMP_SOURCE_ADDR              = 0X40

# Axis remap registers
BNO055_AXIS_MAP_CONFIG_ADDR          = 0X41
BNO055_AXIS_MAP_SIGN_ADDR            = 0X42

# Axis remap values
AXIS_REMAP_X                         = 0x00
AXIS_REMAP_Y                         = 0x01
AXIS_REMAP_Z                         = 0x02
AXIS_REMAP_POSITIVE                  = 0x00
AXIS_REMAP_NEGATIVE                  = 0x01

# SIC registers
BNO055_SIC_MATRIX_0_LSB_ADDR         = 0X43
BNO055_SIC_MATRIX_0_MSB_ADDR         = 0X44
BNO055_SIC_MATRIX_1_LSB_ADDR         = 0X45
BNO055_SIC_MATRIX_1_MSB_ADDR         = 0X46
BNO055_SIC_MATRIX_2_LSB_ADDR         = 0X47
BNO055_SIC_MATRIX_2_MSB_ADDR         = 0X48
BNO055_SIC_MATRIX_3_LSB_ADDR         = 0X49
BNO055_SIC_MATRIX_3_MSB_ADDR         = 0X4A
BNO055_SIC_MATRIX_4_LSB_ADDR         = 0X4B
BNO055_SIC_MATRIX_4_MSB_ADDR         = 0X4C
BNO055_SIC_MATRIX_5_LSB_ADDR         = 0X4D
BNO055_SIC_MATRIX_5_MSB_ADDR         = 0X4E
BNO055_SIC_MATRIX_6_LSB_ADDR         = 0X4F
BNO055_SIC_MATRIX_6_MSB_ADDR         = 0X50
BNO055_SIC_MATRIX_7_LSB_ADDR         = 0X51
BNO055_SIC_MATRIX_7_MSB_ADDR         = 0X52
BNO055_SIC_MATRIX_8_LSB_ADDR         = 0X53
BNO055_SIC_MATRIX_8_MSB_ADDR         = 0X54

# Accelerometer Offset registers
ACCEL_OFFSET_X_LSB_ADDR              = 0X55
ACCEL_OFFSET_X_MSB_ADDR              = 0X56
ACCEL_OFFSET_Y_LSB_ADDR              = 0X57
ACCEL_OFFSET_Y_MSB_ADDR              = 0X58
ACCEL_OFFSET_Z_LSB_ADDR              = 0X59
ACCEL_OFFSET_Z_MSB_ADDR              = 0X5A

# Magnetometer Offset registers
MAG_OFFSET_X_LSB_ADDR                = 0X5B
MAG_OFFSET_X_MSB_ADDR                = 0X5C
MAG_OFFSET_Y_LSB_ADDR                = 0X5D
MAG_OFFSET_Y_MSB_ADDR                = 0X5E
MAG_OFFSET_Z_LSB_ADDR                = 0X5F
MAG_OFFSET_Z_MSB_ADDR                = 0X60

# Gyroscope Offset register s
GYRO_OFFSET_X_LSB_ADDR               = 0X61
GYRO_OFFSET_X_MSB_ADDR               = 0X62
GYRO_OFFSET_Y_LSB_ADDR               = 0X63
GYRO_OFFSET_Y_MSB_ADDR               = 0X64
GYRO_OFFSET_Z_LSB_ADDR               = 0X65
GYRO_OFFSET_Z_MSB_ADDR               = 0X66

# Radius registers
ACCEL_RADIUS_LSB_ADDR                = 0X67
ACCEL_RADIUS_MSB_ADDR                = 0X68
MAG_RADIUS_LSB_ADDR                  = 0X69
MAG_RADIUS_MSB_ADDR                  = 0X6A

# Power modes
POWER_MODE_NORMAL                    = 0X00
POWER_MODE_LOWPOWER                  = 0X01
POWER_MODE_SUSPEND                   = 0X02

# Operation mode settings
OPERATION_MODE_CONFIG                = 0X00
OPERATION_MODE_ACCONLY               = 0X01
OPERATION_MODE_MAGONLY               = 0X02
OPERATION_MODE_GYRONLY               = 0X03
OPERATION_MODE_ACCMAG                = 0X04
OPERATION_MODE_ACCGYRO               = 0X05
OPERATION_MODE_MAGGYRO               = 0X06
OPERATION_MODE_AMG                   = 0X07
OPERATION_MODE_IMUPLUS               = 0X08
OPERATION_MODE_COMPASS               = 0X09
OPERATION_MODE_M4G                   = 0X0A
OPERATION_MODE_NDOF_FMC_OFF          = 0X0B
OPERATION_MODE_NDOF                  = 0X0C

__BNO055_I2C_ADDR = BNO055_ADDRESS_A
__QUATERNION_SCALE = (1.0 / (1 << 14))
__ACCELERATION_SCALE = 100.0
__GYROSCOPE_SCALE = 900.0
__MAGNETOMETER_SCALE = 16.0
__EULER_SCALE = 16.0

_i2c_bus = None
_is_initialized = False


def _set_mode(self, mode):
    """Set operation mode for BNO055 sensor.  Mode should be a value from
    table 3-3 and 3-5 of the datasheet:
      http://www.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
    """
    _i2c_bus.WriteUint8(BNO055_OPR_MODE_ADDR, mode & 0xFF)
    # Delay for 30 milliseconds (datsheet recommends 19ms, but a little more
    # can't hurt and the kernel is going to spend some unknown amount of time
    # too).
    time.sleep(0.03)

def _config_mode(self):
    # Enter configuration mode.
    self.set_mode(OPERATION_MODE_CONFIG)

def _operation_mode(self):
    # Enter operation mode to read sensor data.
    self.set_mode(OPERATION_MODE_NDOF)

def initialize(i2c=None):
    """Initialize the BNO055 sensor.  Must be called once before any other
    BNO055 library functions.  Will return True if the BNO055 was
    successfully initialized, and False otherwise.
    """
    _i2c_bus = i2c

    # First send a thow-away command and ignore any response or I2C errors
    # just to make sure the BNO is in a good state and ready to accept
    # commands (this seems to be necessary after a hard power down).
    try:
        _i2c_bus.WriteUint8(BNO055_PAGE_ID_ADDR, 0)
    except IOError:
        # Swallow an IOError that might be raised by an I2C issue.  Only do
        # this for this very first command to help get the BNO and board's
        # I2C into a clear state ready to accept the next commands.
        pass

    # Make sure we're in config mode and on page 0.
    _config_mode()
    _i2c_bus.WriteUint8(BNO055_PAGE_ID_ADDR, 0)
    # Check the chip ID
    bno_id = _i2c_bus.ReadUint8(BNO055_CHIP_ID_ADDR)
    print('Read chip ID: 0x{0:02X}'.format(bno_id))
    if bno_id != BNO055_ID:
        return False
    '''
    # Reset the device.
    if self._rst is not None:
        # Use the hardware reset pin if provided.
        # Go low for a short period, then high to signal a reset.
        self._gpio.set_low(self._rst)
        time.sleep(0.01)  # 10ms
        self._gpio.set_high(self._rst)
    else:
        # Else use the reset command.  Note that ack=False is sent because
        # the chip doesn't seem to ack a reset in serial mode (by design?).
        self._write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20, ack=False)
    # Wait 650ms after reset for chip to be ready (as suggested in datasheet).
    time.sleep(0.65)
    '''
    # Set to normal power mode.
    _i2c_bus.WriteUint8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)
    # Default to internal oscillator.
    _i2c_bus.WriteUint8(BNO055_SYS_TRIGGER_ADDR, 0x0)
    # Enter normal operation mode.
    _operation_mode()

    _is_initialized = True

def get_revision():
    """Return a tuple with revision information about the BNO055 chip.  Will
    return 5 values:
      - Software revision
      - Bootloader version
      - Accelerometer ID
      - Magnetometer ID
      - Gyro ID
    """
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    # Read revision values.
    accel = _i2c_bus.ReadUint8(BNO055_ACCEL_REV_ID_ADDR)
    mag = _i2c_bus.ReadUint8(BNO055_MAG_REV_ID_ADDR)
    gyro = _i2c_bus.ReadUint8(BNO055_GYRO_REV_ID_ADDR)
    bl = _i2c_bus.ReadUint8(BNO055_BL_REV_ID_ADDR)
    sw_lsb = _i2c_bus.ReadUint8(BNO055_SW_REV_ID_LSB_ADDR)
    sw_msb = _i2c_bus.ReadUint8(BNO055_SW_REV_ID_MSB_ADDR)
    sw = ((sw_msb << 8) | sw_lsb) & 0xFFFF
    # Return the results as a tuple of all 5 values.
    return sw, bl, accel, mag, gyro


def get_system_status(run_self_test=True):
    """Return a tuple with status information.  Three values will be returned:
      - System status register value with the following meaning:
          0 = Idle
          1 = System Error
          2 = Initializing Peripherals
          3 = System Initialization
          4 = Executing Self-Test
          5 = Sensor fusion algorithm running
          6 = System running without fusion algorithms
      - Self test result register value with the following meaning:
          Bit value: 1 = test passed, 0 = test failed
          Bit 0 = Accelerometer self test
          Bit 1 = Magnetometer self test
          Bit 2 = Gyroscope self test
          Bit 3 = MCU self test
          Value of 0x0F = all good!
      - System error register value with the following meaning:
          0 = No error
          1 = Peripheral initialization error
          2 = System initialization error
          3 = Self test result failed
          4 = Register map value out of range
          5 = Register map address out of range
          6 = Register map write error
          7 = BNO low power mode not available for selected operation mode
          8 = Accelerometer power mode not available
          9 = Fusion algorithm configuration error
         10 = Sensor configuration error

    If run_self_test is passed in as False then no self test is performed and
    None will be returned for the self test result.  Note that running a
    self test requires going into config mode which will stop the fusion
    engine from running.
    """
    self_test = None
    if run_self_test:
        # Switch to configuration mode if running self test.
        _config_mode()
        # Perform a self test.
        sys_trigger = _i2c_bus.ReadUint8(BNO055_SYS_TRIGGER_ADDR)
        _i2c_bus.WriteUint8(BNO055_SYS_TRIGGER_ADDR, sys_trigger | 0x1)
        # Wait for self test to finish.
        time.sleep(1.0)
        # Read test result.
        self_test = _i2c_bus.ReadUint8(BNO055_SELFTEST_RESULT_ADDR)
        # Go back to operation mode.
        _operation_mode()
    # Now read status and error registers.
    status = _i2c_bus.ReadUint8(BNO055_SYS_STAT_ADDR)
    error = _i2c_bus.ReadUint8(BNO055_SYS_ERR_ADDR)
    # Return the results as a tuple of all 3 values.
    return status, self_test, error

def get_calibration_status():
    """Read the calibration status of the sensors and return a 4 tuple with
    calibration status as follows:
      - System, 3=fully calibrated, 0=not calibrated
      - Gyroscope, 3=fully calibrated, 0=not calibrated
      - Accelerometer, 3=fully calibrated, 0=not calibrated
      - Magnetometer, 3=fully calibrated, 0=not calibrated
    """
    # Return the calibration status register value.
    cal_status = _i2c_bus.ReadUint8(BNO055_CALIB_STAT_ADDR)
    sys = (cal_status >> 6) & 0x03
    gyro = (cal_status >> 4) & 0x03
    accel = (cal_status >> 2) & 0x03
    mag = cal_status & 0x03
    # Return the results as a tuple of all 3 values.
    return sys, gyro, accel, mag

def get_calibration():
    """Return the sensor's calibration data and return it as an array of
    22 bytes. Can be saved and then reloaded with the set_calibration function
    to quickly calibrate from a previously calculated set of calibration data.
    """
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    _config_mode()
    # Read the 22 bytes of calibration data and convert it to a list (from
    # a bytearray) so it's more easily serialized should the caller want to
    # store it.
    cal_data = list(_i2c_bus.ReadUint8(ACCEL_OFFSET_X_LSB_ADDR, 22))
    # Go back to normal operation mode.
    _operation_mode()

    return cal_data

def set_calibration(self, data):
    """Set the sensor's calibration data using a list of 22 bytes that
    represent the sensor offsets and calibration data.  This data should be
    a value that was previously retrieved with get_calibration (and then
    perhaps persisted to disk or other location until needed again).
    """
    # Check that 22 bytes were passed in with calibration data.
    if data is None or len(data) != 22:
        raise ValueError('Expected a list of 22 bytes for calibration data.')
    # Switch to configuration mode, as mentioned in section 3.10.4 of datasheet.
    _config_mode()
    # Set the 22 bytes of calibration data.
    _i2c_bus.WriteUint8(ACCEL_OFFSET_X_LSB_ADDR, data)
    # Go back to normal operation mode.
    _operation_mode()

def _read_vector(offset, num_values):
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    values = _i2c_bus.read_array(__BNO055_I2C_ADDR, offset, num_values, 'h', big_endian='big')
    return values

def read_orientation():
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    w, x, y, z = _read_vector(BNO055_QUATERNION_DATA_W_LSB_ADDR, 4)
    # Scale values, see 3.6.5.5 in the datasheet.
    return dict(zip(['x', 'y', 'z', 'w'], [x * __QUATERNION_SCALE, y * __QUATERNION_SCALE, z * __QUATERNION_SCALE, w * __QUATERNION_SCALE]))

def read_acceleration():
    """Return the current linear acceleration (acceleration from movement,
    not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
    """
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    x, y, z = _read_vector(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, 3)
    return dict(zip(['x', 'y', 'z'], [x / __ACCELERATION_SCALE, y / __ACCELERATION_SCALE, z / __ACCELERATION_SCALE]))

def read_gyroscope():
    """Return the current gyroscope (angular velocity) reading as a tuple of
    X, Y, Z values in degrees per second.
    """
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    x, y, z = _read_vector(BNO055_GYRO_DATA_X_LSB_ADDR, 3)
    return dict(zip(['x', 'y', 'z'], [x / __GYROSCOPE_SCALE, y / __GYROSCOPE_SCALE, z / __GYROSCOPE_SCALE]))

def read_magnetometer():
    """Return the current magnetometer reading as a tuple of X, Y, Z values
    in micro-Teslas.
    """
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    x, y, z = _read_vector(BNO055_MAG_DATA_X_LSB_ADDR, 3)
    return dict(zip(['x', 'y', 'z'], [x / __MAGNETOMETER_SCALE, y / __MAGNETOMETER_SCALE, z / __MAGNETOMETER_SCALE]))

def read_euler():
    """Return the current absolute orientation as a tuple of heading, roll,
    and pitch euler angles in degrees.
    """
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    yaw, roll, pitch = _read_vector(BNO055_EULER_H_LSB_ADDR, 3)
    return dict(zip(['heading', 'yaw', 'roll', 'pitch'], [yaw / __EULER_SCALE, yaw / __EULER_SCALE, roll / __EULER_SCALE, pitch / __EULER_SCALE]))


def read_temperature():
    """Return the current temperature in Celsius."""
    if not _is_initialized:
        raise HardwareError("Attempt to call methods before {} is initialized".format(__name__))

    return _i2c_bus.ReadInt16(__BNO055_I2C_ADDR, BNO055_TEMP_ADDR)
