import math

ACCELEROMETER_I2C_ADDRESS = 0x1d
MAGNOTOMETER_I2C_ADDRESS = 0x1e

MAG_WHO_AM_I_REG = 0x0F
MAG_CTRL_REG1  = 0x20
MAG_CTRL_REG2  = 0x21
MAG_CTRL_REG3  = 0x22
MAG_CTRL_REG4  = 0x23
MAG_CTRL_REG5  = 0x24
MAG_STATUS_REG = 0x27
MAG_OUTX_L     = 0x28
MAG_OUTX_H     = 0x29
MAG_OUTY_L     = 0x2A
MAG_OUTY_H     = 0x2B
MAG_OUTZ_L     = 0x2C
MAG_OUTZ_H     = 0x2D
MAG_TEMP_OUT_L = 0x2E
MAG_TEMP_OUT_H = 0x2F
MAG_INT_CFG    = 0x30
MAG_INT_SRC    = 0x31
MAG_INT_THS_L  = 0x32
MAG_INT_THS_H  = 0x33

#------------------------------------
#- LSM303C Registers
#------------------------------------
ACC_TEMP_L       = 0x0B
ACC_TEMP_H       = 0x0C
ACC_WHO_AM_I_REG = 0x0F
ACC_ACT_THS      = 0x1E
ACC_ACT_DUR      = 0x1F
ACC_CTRL1        = 0x20
ACC_CTRL2        = 0x21
ACC_CTRL3        = 0x22
ACC_CTRL4        = 0x23
ACC_CTRL5        = 0x24
ACC_CTRL6        = 0x25
ACC_CTRL7        = 0x26
ACC_STATUS       = 0x27
ACC_OUT_X_L      = 0x28
ACC_OUT_X_H      = 0x29
ACC_OUT_Y_L      = 0x2A
ACC_OUT_Y_H      = 0x2B
ACC_OUT_Z_L      = 0x2C
ACC_OUT_Z_H      = 0x2D
ACC_FIFO_CTRL    = 0x2E
ACC_FIFO_SRC     = 0x2F
ACC_IG_CFG1      = 0x30
ACC_IG_SRC1      = 0x31
ACC_IG_THS_X1    = 0x32
ACC_IG_THS_Y1    = 0x33
ACC_IG_THS_Z1    = 0x34
ACC_IG_DUR1      = 0x35
ACC_IG_CFG2      = 0x36
ACC_IG_SRC2      = 0x37
ACC_IG_THS2      = 0x38
ACC_IG_DUR2      = 0x39
ACC_XL_REFERENCE = 0x3A
ACC_XH_REFERENCE = 0x3B
ACC_YL_REFERENCE = 0x3C
ACC_YH_REFERENCE = 0x3D
ACC_ZL_REFERENCE = 0x3E
ACC_ZH_REFERENCE = 0x3F

MAG_TEMP_EN_DISABLE = 0x00
MAG_TEMP_EN_ENABLE  = 0x80

MAG_XYZDA_NO  = 0x00
MAG_XYZDA_YES = 0x08

MAG_DO_0_625_Hz = 0x00
MAG_DO_1_25_Hz  = 0x04
MAG_DO_2_5_Hz   = 0x08
MAG_DO_5_Hz     = 0x0C
MAG_DO_10_Hz    = 0x10
MAG_DO_20_Hz    = 0x14
MAG_DO_40_Hz    = 0x18
MAG_DO_80_Hz    = 0x1C

MAG_FS_4_Ga   =  0x00
MAG_FS_8_Ga   =  0x20
MAG_FS_12_Ga  =  0x40
MAG_FS_16_Ga  =  0x60

MAG_BDU_DISABLE = 0x00
MAG_BDU_ENABLE  = 0x40

MAG_OMXY_LOW_POWER              = 0x00
MAG_OMXY_MEDIUM_PERFORMANCE     = 0x20
MAG_OMXY_HIGH_PERFORMANCE       = 0x40
MAG_OMXY_ULTRA_HIGH_PERFORMANCE = 0x60

MAG_OMZ_LOW_PW                  =  0x00
MAG_OMZ_MEDIUM_PERFORMANCE      =  0x04
MAG_OMZ_HIGH_PERFORMANCE        =  0x08
MAG_OMZ_ULTRA_HIGH_PERFORMANCE  =  0x0C

MAG_MD_CONTINUOUS   = 0x00
MAG_MD_SINGLE       = 0x01
MAG_MD_POWER_DOWN_1 = 0x02
MAG_MD_POWER_DOWN_2 = 0x03

ACC_FS_2g = 0x00
ACC_FS_4g = 0x20
ACC_FS_8g = 0x30

ACC_BDU_DISABLE = 0x00
ACC_BDU_ENABLE  = 0x08

ACC_ODR_POWER_DOWN  = 0x00
ACC_ODR_10_Hz       = 0x10
ACC_ODR_50_Hz       = 0x20
ACC_ODR_100_Hz      = 0x30
ACC_ODR_200_Hz      = 0x40
ACC_ODR_400_Hz      = 0x50
ACC_ODR_800_Hz      = 0x60
ACC_ODR_MASK        = 0x60

ACC_DISABLE_ALL = 0x00
ACC_X_ENABLE    = 0x01
ACC_Y_ENABLE    = 0x02
ACC_Z_ENABLE    = 0x04

ACC_X_NEW_DATA_AVAILABLE    = 0x01
ACC_Y_NEW_DATA_AVAILABLE    = 0x02
ACC_Z_NEW_DATA_AVAILABLE    = 0x04
ACC_ZYX_NEW_DATA_AVAILABLE  = 0x08
ACC_X_OVERRUN               = 0x10
ACC_Y_OVERRUN               = 0x20
ACC_Z_OVERRUN               = 0x40
ACC_ZYX_OVERRUN             = 0x80


__SENSITIVITY_ACC = 0.06103515625   # LSB/mg
__SENSITIVITY_MAG = 0.00048828125   # LSB/Ga

__AXES = ['x', 'y', 'z']

_mag_address = MAGNOTOMETER_I2C_ADDRESS
_acc_address = ACCELEROMETER_I2C_ADDRESS
_i2c_bus = None


def initialize(i2c=None):
    _i2c_bus = i2c

def test():
    pass

def _read_mag_reg(reg):
    return _i2c_bus.ReadUint8(_mag_address, reg)

def _write_mag_reg(reg, value):
    _i2c_bus.WriteUint8(_mag_address, reg, value)

def _read_acc_reg(reg):
    return _i2c_bus.ReadUint8(_acc_address, reg)

def _write_acc_reg(reg, value):
    _i2c_bus.WriteUint8(_acc_address, reg, value)

def _read_acc_values():
    # Note: You may have to resolve endianness by adding a parameter to ReadArray method
    values = _i2c_bus.ReadArray(_acc_address, ACC_OUT_X_H, 6, 'h', endian='big')
    return [value * __SENSITIVITY_ACC for value in values]

def _read_mag_values():
    # Note: You may have to resolve endianness by adding a parameter to ReadArray method
    values = _i2c_bus.ReadArray(_mag_address, MAG_OUTX_H, 6, 'h', endian='big')
    return [value * __SENSITIVITY_ACC for value in values]

def _config_magnetometer():
    value = _read_mag_reg(MAG_WHO_AM_I_REG)
    print("MAG Who Am I: {02:x}".format(value))

    value = _read_mag_reg(MAG_CTRL_REG1)
    value &= ~MAG_DO_80_Hz | 0x03
    value |= MAG_DO_40_Hz
    _write_mag_reg(MAG_CTRL_REG1, value)

    value = _read_mag_reg(MAG_CTRL_REG2)
    value &= ~MAG_FS_16_Ga
    value |= MAG_FS_16_Ga
    _write_mag_reg(MAG_CTRL_REG2, value)

    value = _read_mag_reg(MAG_CTRL_REG5)
    value &= ~MAG_BDU_ENABLE
    value |= MAG_BDU_ENABLE
    _write_mag_reg(MAG_CTRL_REG5, value)

    value = _read_mag_reg(MAG_CTRL_REG1)
    value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE
    value |= MAG_OMXY_HIGH_PERFORMANCE
    _write_mag_reg(MAG_CTRL_REG1, value)

    value = _read_mag_reg(MAG_CTRL_REG4)
    value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE
    value |= MAG_OMZ_HIGH_PERFORMANCE
    _write_mag_reg(MAG_CTRL_REG4, value)

    value = _read_mag_reg(MAG_CTRL_REG3)
    value &= ~MAG_MD_POWER_DOWN_2
    value |= MAG_MD_CONTINUOUS
    _write_mag_reg(MAG_CTRL_REG3, value)

def _config_accelerometer():
    value = _read_acc_reg(ACC_WHO_AM_I_REG)
    print("ACC Who Am I: {02:x}".format(value))

    value = _read_acc_reg(ACC_CTRL4)
    value &= ~ACC_FS_8g
    value |= ACC_FS_2g
    _write_acc_reg(ACC_CTRL4, value)

    value = _read_acc_reg(ACC_CTRL1)
    value &= ~ACC_BDU_ENABLE
    value |= ACC_BDU_ENABLE
    _write_acc_reg(ACC_CTRL1, value)

    value = _read_acc_reg(ACC_CTRL1)
    value &= ~0x07
    value |= ACC_X_ENABLE | ACC_Y_ENABLE | ACC_Z_ENABLE
    _write_acc_reg(ACC_CTRL1, value)

    value = _read_acc_reg(ACC_CTRL1)
    value &= ~ACC_ODR_MASK
    value |= ACC_ODR_100_Hz
    _write_acc_reg(ACC_CTRL1, value)

def _enable_temperature():
    value = _read_mag_reg(MAG_CTRL_REG1)
    value &= ~MAG_TEMP_EN_ENABLE
    value |= MAG_TEMP_EN_ENABLE
    _write_mag_reg(MAG_CTRL_REG1, value)

def _calc_heading(x, y):
    """
    The magnetic compass heading can be determined( in degrees) from the magnetometer’s x and y readings by using
    the following set of equations:
        Direction(y > 0) = 90 - [arcTAN(x / y)] * 180 /pi
        Direction(y < 0) = 270 - [arcTAN(x / y)] * 180 /pi
        Direction(y=0, x < 0) = 180.0
        Direction(y=0, x > 0) = 0.0
    """
    heading = {'r': 0.0, 'd': 0.0}

    atan_xy = math.atan2(x / y)
    pi_over_4 = math.pi / 4

    if y > 0.0:
        heading['r'] = pi_over_4 - atan_xy
        heading['d'] = heading['r'] * 180 / math.pi
        # heading = 90 - (atan_xy * 180) / math.pi

    elif y < 0.0:
        heading['r'] = 3 * pi_over_4 - atan_xy
        # heading = 270 - (math.atan2(x/y) * 180) / math.pi
        heading['d'] = heading['r'] * 180 / math.pi

    elif y == 0.0:
        if x < 0.0:
            heading['r'] = 2 * math.pi
            heading['d'] = 180
        elif x > 0.0:
            heading['r'] = heading['d'] = 0.0

    return heading

def read_imu():
    temp_value = 0
    valueH = 0
    valueL = 0

    imu = {'accel': {}, 'mag': {}, 'temp': {}}

    '''
    Consider changing below to read the block of bytes as 16-bit values
    Q: Are these signed values?
    A: According to the datasheet, the magnatometer values are 2's complement.  The datasheet doesn't say for
    the accelerometer, but why wouldn't acceleration also be 2'c complement.
    '''

    valueH = _read_acc_reg(ACC_OUT_X_H)
    valueL = _read_acc_reg(ACC_OUT_X_L)
    imu['accel']['x'] = ((valueH << 8) | valueL) * __SENSITIVITY_ACC

    valueH = _read_acc_reg(ACC_OUT_Y_H)
    valueL = _read_acc_reg(ACC_OUT_Y_L)
    imu['accel']['y'] = ((valueH << 8) | valueL) * __SENSITIVITY_ACC

    valueH = _read_acc_reg(ACC_OUT_Z_H)
    valueL = _read_acc_reg(ACC_OUT_Z_L)
    imu['accel']['z'] = ((valueH << 8) | valueL) * __SENSITIVITY_ACC

    # Instead of 6 byte reads an combining, do a single 6 byte array read which will convert the bytes into the
    # specified format
    values = _read_acc_values()
    imu['accel'] = dict(zip(__AXES, values))

    valueH = _read_mag_reg(MAG_OUTX_H)
    valueL = _read_mag_reg(MAG_OUTX_L)
    imu['mag']['x'] = ((valueH << 8) | valueL) * __SENSITIVITY_MAG

    valueH = _read_mag_reg(MAG_OUTY_H)
    valueL = _read_mag_reg(MAG_OUTY_L)
    imu['mag']['y'] = ((valueH << 8) | valueL) * __SENSITIVITY_ACC

    valueH = _read_mag_reg(MAG_OUTZ_H)
    valueL = _read_mag_reg(MAG_OUTZ_L)
    imu['mag']['z'] = ((valueH << 8) | valueL) *
    # Instead of 6 byte reads an combining, do a single 6 byte array read which will convert the bytes into the
    # specified format
    values = _read_mag_values()
    imu['mag'] = dict(zip(__AXES, values))

    valueH = _read_mag_reg(MAG_TEMP_OUT_H)
    valueL = _read_mag_reg(MAG_TEMP_OUT_L)
    temp = (valueH << 8) | valueL

    # Alternative
    temp = _i2c_bus.ReadInt16(_mag_address, MAG_TEMP_OUT_H)

    imu['temp']['f'] = (temp * 9.0 / 5.0) + 32.0
    imu['temp']['c'] = (temp - 32) * (5.0 / 9.0)
    # ???? imu['temp']['c'] = temp / 8.0 + 25.0

    imu['heading'] = _calc_heading(imu['mag']['x'], imu['mag']['y'])

    '''
    Returns a dictionary: { 'accel' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                            'mag' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                            'temp': {'f':0.0, 'c':0.0}
                            'heading': {'r':0.0, 'd':0.0}}
    '''
    return imu
