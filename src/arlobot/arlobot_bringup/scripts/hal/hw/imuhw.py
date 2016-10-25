from __future__ import print_function

import math
from lm303chw import *



class ImuHwError(Exception):
    pass


class ImuHw:

    __SENSITIVITY_ACC = 0.06103515625   # LSB/mg
    __SENSITIVITY_MAG = 0.00048828125   # LSB/Ga

    ACC_ADDR = ACCELEROMETER_I2C_ADDRESS
    MAG_ADDR = MAGNOTOMETER_I2C_ADDRESS

    __AXES = ['x', 'y', 'z']

    def __init__(self, i2cbus, mag_address, acc_address):
        self._mag_address = mag_address
        self._acc_address = acc_address

        self._i2c_bus = i2cbus
        self._initialize()

    def _read_mag_reg(self, reg):
        return self._i2c_bus.ReadUint8(self._mag_address, reg)

    def _write_mag_reg(self, reg, value):
        self._i2c_bus.WriteUint8(self._mag_address, reg, value)

    def _read_acc_reg(self, reg):
        return self._i2c_bus.ReadUint8(self._acc_address, reg)

    def _write_acc_reg(self, reg, value):
        self._i2c_bus.WriteUint8(self._acc_address, reg, value)

    def _read_acc_values(self):
        # Note: You may have to resolve endianness by adding a parameter to ReadArray method
        values = self._i2c_bus.ReadArray(self._acc_address, ACC_OUT_X_H, 6, 'h', endian='big')
        return [value * self.__SENSITIVITY_ACC for value in values]

    def _read_mag_values(self):
        # Note: You may have to resolve endianness by adding a parameter to ReadArray method
        values = self._i2c_bus.ReadArray(self._mag_address, MAG_OUTX_H, 6, 'h', endian='big')
        return [value * self.__SENSITIVITY_ACC for value in values]

    def _config_magnetometer(self):
        value = self._read_mag_reg(MAG_WHO_AM_I_REG)
        print("MAG Who Am I: {02:x}".format(value))

        '''
        //successes += MAG_SetODR(modr);
        MAG_ReadReg(MAG_CTRL_REG1, &value);
        value &= ~(MAG_DO_80_Hz | 0x03);
        value |= MAG_DO_40_Hz;
        MAG_WriteReg(MAG_CTRL_REG1, value);
        //MAG_ReadReg(MAG_CTRL_REG1, &test_value);
        //print("\t\t%02x == %02x -> %s\n", value, test_value, value == test_value ? "TRUE" : "FALSE");
        '''
        value = self._read_mag_reg(MAG_CTRL_REG1)
        value &= ~MAG_DO_80_Hz | 0x03
        value |= MAG_DO_40_Hz
        self._write_mag_reg(MAG_CTRL_REG1, value)

        '''
        // Initialize magnetic field full scale
        //successes += MAG_SetFullScale(mfs);
        MAG_ReadReg(MAG_CTRL_REG2, &value);
        value &= ~MAG_FS_16_Ga; //mask
        value |= MAG_FS_16_Ga;
        MAG_WriteReg(MAG_CTRL_REG2, value);
        '''
        value = self._read_mag_reg(MAG_CTRL_REG2)
        value &= ~MAG_FS_16_Ga
        value |= MAG_FS_16_Ga
        self._write_mag_reg(MAG_CTRL_REG2, value)

        '''
        // Enabling block data updating
        //successes += MAG_BlockDataUpdate(mbu);
        MAG_ReadReg(MAG_CTRL_REG5, &value);
        //print("MAG_CTRL_REG5 (%02x), %02x\n", MAG_CTRL_REG5, value);
        value &= ~MAG_BDU_ENABLE; //mask
        value |= MAG_BDU_ENABLE;
        //print("MAG_CTRL_REG5 (%02x), %02x\n", MAG_CTRL_REG5, value);
        MAG_WriteReg(MAG_CTRL_REG5, value);
        '''
        value = self._read_mag_reg(MAG_CTRL_REG5)
        value &= ~MAG_BDU_ENABLE
        value |= MAG_BDU_ENABLE
        self._write_mag_reg(MAG_CTRL_REG5, value)

        '''
        // Initialize magnetometer X/Y axes output data rate
        //successes += MAG_XY_AxOperativeMode(mxyodr);
        MAG_ReadReg(MAG_CTRL_REG1, &value);
        value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
        value |= MAG_OMXY_HIGH_PERFORMANCE;
        MAG_WriteReg(MAG_CTRL_REG1, value);
        '''
        value = self._read_mag_reg(MAG_CTRL_REG1)
        value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE
        value |= MAG_OMXY_HIGH_PERFORMANCE
        self._write_mag_reg(MAG_CTRL_REG1, value)

        '''
        // Initialize magnetometer Z axis performance mode
        //successes += MAG_Z_AxOperativeMode(mzodr);
        MAG_ReadReg(MAG_CTRL_REG4, &value);
        value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
        value |= MAG_OMZ_HIGH_PERFORMANCE;
        MAG_WriteReg(MAG_CTRL_REG4, value);
        '''
        value = self._read_mag_reg(MAG_CTRL_REG4)
        value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE
        value |= MAG_OMZ_HIGH_PERFORMANCE
        self._write_mag_reg(MAG_CTRL_REG4, value)

        '''
        // Initialize magnetometer run mode.
        //successes += MAG_SetMode(mm);
        MAG_ReadReg(MAG_CTRL_REG3, &value);
        value &= ~MAG_MD_POWER_DOWN_2;
        value |= MAG_MD_CONTINUOUS;
        MAG_WriteReg(MAG_CTRL_REG3, value);
        //print("\tComplete\n");
        '''
        value = self._read_mag_reg(MAG_CTRL_REG3)
        value &= ~MAG_MD_POWER_DOWN_2
        value |= MAG_MD_CONTINUOUS
        self._write_mag_reg(MAG_CTRL_REG3, value)

    def _config_accelerometer(self):

        value = self._read_acc_reg(ACC_WHO_AM_I_REG)
        print("ACC Who Am I: {02:x}".format(value))

        '''
        ////////// Initialize Accelerometer //////////
        // Initialize acceleration full scale
        //successes += ACC_SetFullScale(afs);
        ACC_ReadReg(ACC_CTRL4, &value);
        value &= ~ACC_FS_8g;
        value |= ACC_FS_2g;
        ACC_WriteReg(ACC_CTRL4, value);
        '''
        value = self._read_acc_reg(ACC_CTRL4)
        value &= ~ACC_FS_8g
        value |= ACC_FS_2g
        self._write_acc_reg(ACC_CTRL4, value)

        '''
        // Enable block data updating
        //successes += ACC_BlockDataUpdate(abu);
        ACC_ReadReg(ACC_CTRL1, &value);
        value &= ~ACC_BDU_ENABLE;
        value |= ACC_BDU_ENABLE;
        ACC_WriteReg(ACC_CTRL1, value);
        '''
        value = self._read_acc_reg(ACC_CTRL1)
        value &= ~ACC_BDU_ENABLE
        value |= ACC_BDU_ENABLE
        self._write_acc_reg(ACC_CTRL1, value)

        '''
        // Enable X, Y, and Z accelerometer axes
        //successes += ACC_EnableAxis(aea);
        ACC_ReadReg(ACC_CTRL1, &value);
        value &= ~0x07;
        value |= ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE;
        ACC_WriteReg(ACC_CTRL1, value);
        '''
        value = self._read_acc_reg(ACC_CTRL1)
        value &= ~0x07
        value |= ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE
        self._write_acc_reg(ACC_CTRL1, value)

        '''
        // Initialize accelerometer output data rate
        //successes += ACC_SetODR(aodr);
        ACC_ReadReg(ACC_CTRL1, &value);
        value &= ~ACC_ODR_MASK;
        value |= ACC_ODR_100_Hz;
        ACC_WriteReg(ACC_CTRL1, value);

        //print("\tComplete\n");
        '''
        value = self._read_acc_reg(ACC_CTRL1)
        value &= ~ACC_ODR_MASK
        value |= ACC_ODR_100_Hz
        self._write_acc_reg(ACC_CTRL1, value)

        pass

    def _enable_temperature(self):

        '''
        //print("Enable Temperature Sensor\n");
        // Enable temperature sensor
        MAG_ReadReg(MAG_CTRL_REG1, &value);
        value &= ~MAG_TEMP_EN_ENABLE; //mask
        value |= MAG_TEMP_EN_ENABLE;
        MAG_WriteReg(MAG_CTRL_REG1, value);

        //print("\tComplete\n");
        //print("Complete!\n");
        '''
        value = self._read_mag_reg(MAG_CTRL_REG1)
        value &= ~MAG_TEMP_EN_ENABLE
        value |= MAG_TEMP_EN_ENABLE
        self._write_mag_reg(MAG_CTRL_REG1, value)

    def _initialize(self):

        # Configure the magnetometer
        self._config_magnetometer()
        self._config_accelerometer()
        self._enable_temperature()

    def _calc_heading(self, x, y):
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
            #heading = 90 - (atan_xy * 180) / math.pi

        elif y < 0.0:
            heading['r'] = 3 * pi_over_4 - atan_xy
            #heading = 270 - (math.atan2(x/y) * 180) / math.pi
            heading['d'] = heading['r'] * 180 / math.pi

        elif y == 0.0:
            if x < 0.0:
                heading['r'] = 2 * math.pi
                heading['d'] = 180
            elif x > 0.0:
                heading['r'] = heading['d'] = 0.0

        return heading

    def _read_imu(self):
        temp_value = 0
        valueH = 0
        valueL = 0

        imu = {'accel' : {}, 'mag' : {}, 'temp' : {}}

        '''
        Consider changing below to read the block of bytes as 16-bit values
        Q: Are these signed values?
        A: According to the datasheet, the magnatometer values are 2's complement.  The datasheet doesn't say for
        the accelerometer, but why wouldn't acceleration also be 2'c complement.
        '''

        '''
        ACC_ReadReg(ACC_OUT_X_H, &valueH);
        ACC_ReadReg(ACC_OUT_X_L, &valueL);
        temp_value = (valueH << 8) | valueL;
        state->accel_x = temp_value * SENSITIVITY_ACC;
        '''
        valueH = self._read_acc_reg(ACC_OUT_X_H)
        valueL = self._read_acc_reg(ACC_OUT_X_L)
        imu['accel']['x'] = ( (valueH << 8) | valueL ) * self.__SENSITIVITY_ACC

        '''
        ACC_ReadReg(ACC_OUT_Y_H, &valueH);
        ACC_ReadReg(ACC_OUT_Y_L, &valueL);
        temp_value = (valueH << 8) | valueL;
        state->accel_y = temp_value * SENSITIVITY_ACC;
        '''
        valueH = self._read_acc_reg(ACC_OUT_Y_H)
        valueL = self._read_acc_reg(ACC_OUT_Y_L)
        imu['accel']['y'] = ( (valueH << 8) | valueL ) * self.__SENSITIVITY_ACC

        '''
        ACC_ReadReg(ACC_OUT_Z_H, &valueH);
        ACC_ReadReg(ACC_OUT_Z_L, &valueL);
        temp_value = (valueH << 8) | valueL;
        state->accel_z = temp_value * SENSITIVITY_ACC;
        '''
        valueH = self._read_acc_reg(ACC_OUT_Z_H)
        valueL = self._read_acc_reg(ACC_OUT_Z_L)
        imu['accel']['z'] =  ( (valueH << 8) | valueL) * self.__SENSITIVITY_ACC


        # Instead of 6 byte reads an combining, do a single 6 byte array read which will convert the bytes into the
        # specified format
        values = self._read_acc_values()
        imu['accel'] = dict(zip(self.__AXES, values))


        '''
        MAG_ReadReg(MAG_OUTX_L, &valueL);
        MAG_ReadReg(MAG_OUTX_H, &valueH);
        temp_value = (valueH << 8) | valueL;
        state->mag_x = temp_value * SENSITIVITY_MAG;
        '''
        valueH = self._read_mag_reg(MAG_OUTX_H)
        valueL = self._read_mag_reg(MAG_OUTX_L)
        imu['mag']['x'] = ( (valueH << 8) | valueL) * self.__SENSITIVITY_MAG

        '''
        MAG_ReadReg(MAG_OUTY_L, &valueL);
        MAG_ReadReg(MAG_OUTY_H, &valueH);
        temp_value = (valueH << 8) | valueL;
        state->mag_y = temp_value * SENSITIVITY_MAG;
        '''
        valueH = self._read_mag_reg(MAG_OUTY_H)
        valueL = self._read_mag_reg(MAG_OUTY_L)
        imu['mag']['y'] = ( (valueH << 8) | valueL) * self.__SENSITIVITY_ACC

        '''
        MAG_ReadReg(MAG_OUTZ_L, &valueL);
        MAG_ReadReg(MAG_OUTZ_H, &valueH);
        temp_value = (valueH << 8) | valueL;
        state->mag_z = temp_value * SENSITIVITY_MAG;
        '''
        valueH = self._read_mag_reg(MAG_OUTZ_H)
        valueL = self._read_mag_reg(MAG_OUTZ_L)
        imu['mag']['z'] = ( (valueH << 8) | valueL) * self.__SENSITIVITY_ACC


        # Instead of 6 byte reads an combining, do a single 6 byte array read which will convert the bytes into the
        # specified format
        values = self._read_mag_values()
        imu['mag'] = dict(zip(self.__AXES, values))

        '''
        #MAG_ReadReg(MAG_TEMP_OUT_L, &valueL);
        #MAG_ReadReg(MAG_TEMP_OUT_H, &valueH);
        #// Note: tempurature values is represented as a 2's complement value
        #temp_value = (valueH << 8) | valueL;
        #// 8 digits/C
        #// Reads 0 @ 25C
        #state->temp_c = (temp_value/8.0) + 25.0;
        #state->temp_f = (state->temp_c * 9.0 / 5.0) + 32.0;
        '''
        valueH = self._read_mag_reg(MAG_TEMP_OUT_H)
        valueL = self._read_mag_reg(MAG_TEMP_OUT_L)
        temp = (valueH << 8) | valueL

        # Alternative
        temp = self._i2c_bus.ReadInt16(self.MAG_ADDR, MAG_TEMP_OUT_H)

        imu['temp']['f'] = (temp * 9.0 / 5.0) + 32.0
        imu['temp']['c'] = (temp - 32) * (5.0 / 9.0)
        # ???? imu['temp']['c'] = temp / 8.0 + 25.0

        imu['heading'] = self._calc_heading(imu['mag']['x'], imu['mag']['y'])

        '''
        Returns a dictionary: { 'accel' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'mag' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'temp': {'f':0.0, 'c':0.0}
                                'heading': {'r':0.0, 'd':0.0}}
        '''
        return imu

    # Use the cpp library for info on how to read accel, mag, and temp data from the IMU
    def GetAccel(self):
        imu = self._read_imu()
        return imu["accel"]

    def GetMag(self):
        imu = self._read_imu()
        return imu['mag']

    def GetTemp(self):
        imu = self._read_imu()
        return imu['temp']

    def GetImuData(self):
        return self._read_imu()