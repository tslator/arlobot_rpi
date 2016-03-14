from __future__ import print_function

# This module provides access to the Raspberry Pi SenseHat features

from sense_hat import SenseHat

sense = SenseHat()

class DisplayError(Exception):
    pass


class Display:
    def __init__(self, sense_hat):
        self._sense_hat = sense_hat


    def SetRotation(self, rotate, redraw=True):
        """
        SetRotation - sets the orientation of the display
        :param rotate: degrees of rotation: 0, 90, 180, 270 are valid values
        :param redraw: redraw the display (default is True)
        :return: None
        """
        self._sense_hat.set_rotation(rotate, redraw)


    def FlipHorizontal(self, redraw=True):
        """
        FlipHorizontal - flip the display in the horizontal direction
        :param redraw: redraw the display (default is True)
        :return: None
        """
        self._sense_hat.flip_h(redraw)


    def FlipVertical(self, redraw):
        """
        FlipVertical - flip the display in the vertical direction
        :param redraw: redraw the display (default is True)
        :return: None
        """
        self._sense_hat.flip_v(redraw)


    def SetPixels(self, pixel_list):
        """
        SetPixels - updates entire display with values in pixel list
        :param pixel_list: list of pixels to be displayed
        :return: None
        """
        self._sense_hat.set_pixels(pixel_list)


    def GetPixels(self):
        """
        GetPixels - returns the current pixel values of the display
        :return: current pixel values of the display
        """
        return self._sense_hat.get_pixels()


    def SetPixel(self, x, y, *pixels):
        """
        SetPixel - sets a pixel value at position x, y
        :param x: x position
        :param y: y position
        :param pixels: either a tuple, (r, g, b) or individual parameters, r, g, b
        :return: None
        """
        self._sense_hat.set_pixel(x, y, *pixels)


    def GetPixel(self, x, y):
        """
        GetPixel - returns the pixel value at position x, y
        :param x: x position
        :param y: y position
        :return: either a tuple, (r, g, b) or individual parameters, r, g, b
        """
        return self._sense_hat.get_pixel(x, y)


    def LoadImage(self, file_path, redraw=True):
        """
        LoadImage - loads an 8x8 image into the display
        :param file_path: path to the image file
        :param redraw: redraw the display (default is True)
        :return: converted image
        """
        return self._sense_hat.load_image(file_path, redraw)


    def Clear(self, *color):
        """
        Clear - sets the entire display to a specific color
        :param color: either a tuple, (r, g, b) or individual parameters, r, g, b
        :return: None
        """
        self._sense_hat.clear(*color)



class EnvironmentError(Exception):
    pass


class Environment:
    def __init__(self, sense_hat):
        self._sense_hat = sense_hat


    def GetHumidity(self):
        """
        GetHumidity - the percentage of relative humidity from the humidity sensor
        :return: percentage of relative humidity
        """
        return self._sense_hat.get_humidity()


    def GetTemperature(self, celcius_or_fahrenheit=False):
        """
        GetTemperature - the current temperature in degrees Celsius from the humidity sensor
        :param celcius_or_fahrenheit:
        :return: temperature value
        """
        temp = self._sense_hat.get_temperature()
        if not celcius_or_fahrenheit:
            temp = ((temp * 9) / 5) + 32

        return temp


    def GetPressure(self):
        """
        GetPressure - returns the current pressure in Millibars from the pressure sensor
        :return: pressure in millibars
        """
        return self._sense_hat.get_pressure()

class ImuError(Exception):
    pass


class Imu:
    def __init__(self, sense_hat, compass=True, gyro=True, accel=True):
        self._sense_hat = sense_hat
        self._sense_hat.set_imu_config(compass, gyro, accel)

    def GetOrientation(self, degrees_or_radians=True):
        """
        GetOrientation - returns the current orientation (radians or degrees) in axes of pitch, roll and yaw.

        :param degrees_or_radians: True (default) returns in degrees, False returns in radians
        :return: dictionary of values: pitch, roll, and yaw
        """
        orientation = None
        if degrees_or_radians:
            orientation = self._sense_hat.get_orientation_degrees()
        else:
            orientation = self._sense_hat.get_orientation_radians()

        return orientation

    def GetCompass(self):
        """
        GetCompass - returns the direction of North from the magnetometer in degrees
        :return: North direction in degrees
        """
        return self._sense_hat.get_compass()


    def GetCompassXYZ(self):
        """
        GetCompassXYZ - returns the raw x, y and z axis magnetometer data
        :return: dictionary of values: x, y, z
        """
        return self._sense_hat.get_compase_raw()


    def GetGyroscope(self):
        """
        GetGyroscope - returns the current orientation from the gyroscope in pitch, roll, yaw format
        :return: dictionary of values: pitch, roll, yaw
        """
        return self._sense_hat.get_gyroscope()


    def GetGyroscopeXYZ(self):
        """
        GetGyroscopeXYZ - returns the current orientation from the gyroscope in x, y, z format
        :return: dictionary of values: x, y, z
        """
        return self._sense_hat.get_gyroscope_raw()

    def GetAccelerometer(self):
        """
        GetAcclerometer - returns the current orientation from the accelerometer in pitch, roll, yaw format
        :return: dictionary of values: pitch, roll, yaw
        """
        return self._sense_hat.get_accelerometer()

    def GetAccelerometerXYZ(self):
        """
        GetAccelerometerXYZ - returns the current orientation from the accelerometer in x, y, z format
        :return: dictionary of values: x, y, z
        """
        self._sense_hat.get_accelerometer_raw()



