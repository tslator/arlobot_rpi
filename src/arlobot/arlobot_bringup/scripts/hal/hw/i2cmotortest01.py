from __future__ import print_function

from i2c import I2CBus, I2CBusError
import sys
import time

if __name__ == "__main__":
<<<<<<< HEAD
    print("i2cmotortest01")
    #try:
    #    i2c1 = I2CBus(I2CBus.DEV_I2C_1)
    #except I2CBusError:
    #    print("Error instantiating I2CBus")

    #print("successfully created i2c bus")

    #speed = int(sys.argv[1])

    #print("speed: ", speed)
    #right_start = i2c1.ReadInt32(0x08, 8)
    #i2c1.WriteInt16(0x08, 2, speed)
=======
    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
    except I2CBusError:
        print("Error instantiating I2CBus")

    print("successfully created i2c bus")
    speed = int(sys.argv[1])

    print("speed: ", speed)
    right_start = i2c1.ReadFloat(0x08, 0)
    left_start = i2c1.ReadInt32(0x08, )
    #i2c1.WriteInt16(0x08, 2, speed)
    #i2c1.WriteInt16(0x09, 2, speed)
>>>>>>> 12b5873b0859d610b664a64a0b5dcc66c8c6b016

    #start_time = time.time()

    #time.sleep(float(sys.argv[2]))

    #right_velocity = int(i2c1.ReadInt16(0x08, 12))
<<<<<<< HEAD
    #i2c1.WriteInt16(0x08, 2, 0)
    #end_time = time.time()
    #right_end = i2c1.ReadInt32(0x08, 8)
=======
    #left_velocity = int(i2c1.ReadInt16(0x09, 12))
    #i2c1.WriteInt16(0x08, 2, 0)
    #i2c1.WriteInt16(0x09, 2, 0)
    #end_time = time.time()
    #right_end = i2c1.ReadInt32(0x08, 8)
    #left_end = i2c1.ReadInt32(0x09, 8)
>>>>>>> 12b5873b0859d610b664a64a0b5dcc66c8c6b016
    #print("distance(m): ", (right_end - right_start) * (end_time - start_time)/1000, " velocity(m/s): ", right_velocity)
    #print("distance(m): ", (left_end - left_start) * (end_time - start_time)/1000, " velocity(m/s): ", left_velocity)
    

