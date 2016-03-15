import thread, time, sys, traceback, math, serial


class Xv11HwError(Exception):
    pass


class Xv11Hw:

    PORT = "/dev/ttyACM0"
    BAUD = 115200


    def __init__(self, port, baud):
        self._port = port
        self._baud = baud

        # Need to connect to a serial gateway server with a worker thread and register a callback function
        self._ranges = []
        self._intensities = []
        self._time_increment = 0

        # Create a serial port and thread for reading the lidar data
        ser = serial.Serial(self._port, self._baud)
        th = thread.start_new_thread(self.read_Lidar, ())

    def _callback(self, data):
        """
        Read data from serial port and parse out ranges, intensities, and time_increment from the data
        :param data:
        :return:
        """

        # Add a lock so that accesses are serialized
        self._ranges = 0
        self._intensities = 0
        self._tiime_increment = 0

        self.init_level = 0
        self.index = 0

    def checksum(self, data):
        """Compute and return the checksum as an int.
        
        data -- list of 20 bytes (as ints), in the order they arrived in.
        """
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
        # compute the checksum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int( checksum )

    def compute_speed(self, data):
        speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
        return speed_rpm

    def read_Lidar(self):
        
        nb_errors = 0
        while True:
            try:
                time.sleep(0.00001) # do not hog the processor power

                if self.init_level == 0 :
                    b = ord(ser.read(1))
                    # start byte
                    if b == 0xFA :
                        self.init_level = 1
                        #print lidarData
                    else:
                        self.init_level = 0
                elif self.init_level == 1:
                    # position index
                    b = ord(ser.read(1))
                    if b >= 0xA0 and b <= 0xF9 :
                        self.index = b - 0xA0
                        self.init_level = 2
                    elif b != 0xFA:
                        self.init_level = 0
                elif self.init_level == 2 :
                    # speed
                    b_speed = [ ord(b) for b in ser.read(2)]
                    
                    # data
                    b_data0 = [ ord(b) for b in ser.read(4)]
                    b_data1 = [ ord(b) for b in ser.read(4)]
                    b_data2 = [ ord(b) for b in ser.read(4)]
                    b_data3 = [ ord(b) for b in ser.read(4)]

                    # for the checksum, we need all the data of the packet...
                    # this could be collected in a more elegent fashion...
                    all_data = [ 0xFA, self.index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                    # checksum
                    b_checksum = [ ord(b) for b in ser.read(2) ]
                    incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                    # verify that the received checksum is equal to the one computed from the data
                    if self.checksum(all_data) == incoming_checksum:
                        speed_rpm = self.compute_speed(b_speed)
                        #if visualization:
                        #    gui_update_speed(speed_rpm)
                        
                        #update_view(index * 4 + 0, b_data0)
                        #update_view(index * 4 + 1, b_data1)
                        #update_view(index * 4 + 2, b_data2)
                        #update_view(index * 4 + 3, b_data3)
                    else:
                        # the checksum does not match, something went wrong...
                        nb_errors +=1
                        #if visualization:
                        #    label_errors.text = "errors: "+str(nb_errors)
                        
                        # display the samples in an error state
                        #update_view(index * 4 + 0, [0, 0x80, 0, 0])
                        #update_view(index * 4 + 1, [0, 0x80, 0, 0])
                        #update_view(index * 4 + 2, [0, 0x80, 0, 0])
                        #update_view(index * 4 + 3, [0, 0x80, 0, 0])
                        
                    self.init_level = 0 # reset and wait for the next packet
                    
                else: # default, should never happen...
                    self.init_level = 0
            except :
                traceback.print_exc(file=sys.stdout)


    def GetLaserScan(self):
        """
        """

        # Add a lock so that accesses are serialized
        return self._ranges, self._intensities, self._time_increment


if __name__ == "__main__":
    xv11 = Xv11Hw(Xv11Hw.PORT, Xv11Hw.BAUD)
