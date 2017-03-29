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


# Create a class that is a protocol handler where you register callbacks that trigger on specific serial data
# For example, a callback for when the prompt occurs:
#       r'Make an entry \[[a-z]-[a-z],x\/X\]:.*'
# The callback can be triggered when a regular expression matches or a glob, etc.

# The callbacks will be called from the thread that reads from the serial port?
# We could make the serial server dumber so that all it can do is read and write data to the serial port, and
# every time there is data read, it calls a callback in the protocol handler.  The protocol handler determines
# if there has been a match rather than the serial server.  Remember the serial server, as is, was designed to
# be a line protocol detector.  We need more and so we can move the 'line detector' to the protocol handler and
# simplify the serial server

# The serial server will lose the StringIO handling and just pass serial data as it is received from the serial port
# to the protocol handler.from

# The protocol handler will attach a callback to the serial server, to receive data.  In that the serial data receive
# callback, it will take over the serial server task of adding the data to StringIO and then attempt to match the data
# against registered 'events', e.g., "end menu display", "selection prompt", "data entry prompt", etc.

# The protocol handler is the proper place to have context.  In fact, we should be able to subclass the protocol handler
# into a class that customizes for handling display, calibration, validation, etc.
class ProtocolHandler():
    def __init__(self, server):
        self._callbacks = {}
        self._server = server
        self._server.add_callback(self._recv_data)
        self._events = {}

    def _recv_data(self, data):
        """
        :description: Add the data received for the StringIO object then for each registered event apply the filter and
        call the callback if it matches
        :param data:
        :return:
        """
        self._string_io.write(data)

        recv_data = self._string_io.getvalue()
        for name, event in self._events:
            if re.match(event.regex, recv_data):
                event.callback(name, recv_data)
                self._string_io.close()
                self._string_io = StringIO()

    def start(self):
        self._server.Start()

    def stop(self):
        self._server.Stop()

    def add_callback(self, name, regex, callback):
        # Note: Per Fluent Python, store a named tuple instead of a plain tuple
        self._events[name] = (regex, callback)

    def execute(self, menu, item):
        pass


class DisplayHandler(ProtocolHandler):
    def __init__(self, serial):
        ProtocolHandler.__init__(self, serial)

        self.add_event('select_prompt', r'Make an entry \[[a-z]-[a-z],x\/X\]:.*', self._select_prompt)

        self.start()

    def _select_prompt(self, name, data):
        pass

    def get_motor_calibration(self, mode='raw'):
        """
        :description: Request and retrieve the display data for the motor calibration.  This is a blocking call.
        :return:
        """
        self.execute('d', 'a')
        return self._data

dh = DisplayHandler(serial=serial(port='/dev/ttyACM0', baud=115200))
raw_data = dh.get_motor_calibration()
parsed_data = dh.get_motor_calibration(mode='parsed')


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
