#!/usr/bin/env python
'''
Created on November 20, 2010

@author: Dr. Rainer Hessmer
'''
import threading
import serial
from cStringIO import StringIO
import time
import rospy
import re

data_buffer = []
finished_count = 0
prompt_received = True

def _OnLineReceived(line):
    #global finished_count
    global data_buffer
    data_buffer.append(line)
    #print line
    #if 'exit' in line:
    #    finished_count += 1
    #print finished_count

def _PromptReceived(line):
     global data_buffer
     print line
     data_buffer.append(line)
     prompt_received = True
     print prompt_received

class SerialServer(object):
    '''
    Helper class for receiving lines from a serial port
    '''

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, lineHandler=_OnLineReceived):
        '''
        Initializes the receiver class.
        port: The serial port to listen to.
        receivedLineHandler: The function to call when a line was received.
        '''
        self._Port = port
        self._Baudrate = baudrate
        self.ReceivedLineHandler = lineHandler
        self._KeepRunning = False

    def Start(self):
        try:
            self._Serial = serial.Serial(port=self._Port, baudrate=self._Baudrate, timeout=1)
        except:
            rospy.loginfo("SERIAL PORT Start Error")
            raise
        self._KeepRunning = True
        self._ReceiverThread = threading.Thread(target=self._Listen)
        self._ReceiverThread.setDaemon(True)
        self._ReceiverThread.start()

    def Stop(self):
        rospy.loginfo("Stopping serial gateway")
        self._KeepRunning = False
        time.sleep(.1)
        try:
            self._Serial.close()
        except:
            rospy.loginfo("SERIAL PORT Stop Error")
            raise

    def _Listen(self):
        stringIO = StringIO()
        while self._KeepRunning:
            try:
                data = self._Serial.read()
            except:
                rospy.loginfo("SERIAL PORT Listen Error")
                raise
            if data == '\r':
                pass
            if data == '\n':
                self.ReceivedLineHandler(stringIO.getvalue())
                stringIO.close()
                stringIO = StringIO()
            else:
                stringIO.write(data)

                if re.match(r'Make an entry \[[a-z]-[a-z],x\/X\]:.*', stringIO.getvalue()):
                    _PromptReceived(stringIO.getvalue())
                    stringIO.close()
                    stringIO = StringIO()    

    def Write(self, data):
        #AttributeError: 'SerialDataGateway' object has no attribute '_Serial'
        try:
            self._Serial.write(data)
        except AttributeError:
            rospy.loginfo("SERIAL PORT Write Error")
            raise

if __name__ == '__main__':
    def wait():
        #global finished_count
        global prompt_received
        #while finished_count < 2:
        while not prompt_received:
            print prompt_received
            time.sleep(0.001)
    
    def reset():
        global finished_count
        global data_buffer
        global prompt_received

        finished_count = 0
        data_buffer = []
        prompt_received = False

    def execute(srv, menu, item):
        print ("reset ...")
        reset()
        print ("select menu {} ...".format(menu))
        srv.Write(menu)
        print ("select item {} ...".format(item))
        srv.Write(item)
        print ("Wait for prompt ...")
        wait()
        print ("Prompt received")
        srv.Write('x')
        print ("Exiting menu")
        print data_buffer

    def display_left_motor_calibration(srv):
        execute(srv, 'd', 'a')

    def display_right_motor_calibration(srv):
        execute(srv, 'd', 'b')

    def display_pid_gains(srv):
        execute(srv, 'd', 'c')
        
    def display_linear_angular_bias(srv):
        execute(srv, 'd', 'e')

    def display_all(srv):
        execute(srv, 'd', 'f')

    def validate_motors(srv):
        execute(srv, 'v', 'a')

    def validate_pids(srv):
        execute(srv, 'v', 'b')

    def validate_left_pid_forward(srv):
        execute(srv, 'v', 'c')

    def validate_left_pid_backward(srv):
        execute(srv, 'v', 'e')

    def validate_right_pid_forward(srv):
        execute(srv, 'v', 'f')

    def validate_right_pid_backward(srv):
        execute(srv, 'v', 'g')

    def validate_straight_line_forward(srv):
        execute(srv, 'v', 'h')

    def validate_straight_line_backward(srv):
        execute(srv, 'v', 'i')

    def validate_rotate_cw(srv):
        execute(srv, 'v', 'j')

    def validate_rotate_ccw(srv):
        execute(srv, 'v', 'k')

    def calibrate_motors(srv):
        execute(srv, 'c', 'a')

    dataReceiver = SerialServer("/dev/ttyACM0", 115200)
    dataReceiver.Start()

    # Display
    
    display_left_motor_calibration(dataReceiver)
    print "\n"
    '''
    display_right_motor_calibration(dataReceiver)
    print "\n"
    
    display_pid_gains(dataReceiver)
    print "\n"

    display_linear_angular_bias(dataReceiver)
    print "\n"

    display_all(dataReceiver)
    print "\n"
    '''
 
    # Validation
    '''
    validate_motors(dataReceiver)
    print "\n"
    
    validate_pids(dataReceiver)
    print "\n"

    validate_left_pid_forward(dataReceiver)
    print "\n"

    validate_left_pid_backward(dataReceiver)
    print "\n"

    validate_right_pid_forward(dataReceiver)
    print "\n"
 
    validate_right_pid_backward(dataReceiver)
    print "\n"

    validate_straight_line_forward(dataReceiver)
    print "\n"

    validate_straight_line_backward(dataReceiver)
    print "\n"

    validate_rotate_cw(dataReceiver)
    print "\n"

    validate_rotate_ccw(dataReceiver)
    print "\n"
    '''

    # Calibration
    #calibrate_motors(dataReceiver)
    #print "\n" 

    dataReceiver.Stop()
