#! /usr/bin/env python
import serial
import time
import struct
import numpy as np

import rospy
from arduino_control.msg import UInt8Array

class ArduinoController():

    def __init__(self, timeout = .25):
        rospy.init_node('arduino_control')
        self.ser = None
        self.baudrate = rospy.get_param('~baudrate')
        self.port = rospy.get_param('~port')

        self.msg_sub = rospy.Subscriber('/arduino/msgout', UInt8Array, self.msg_callback, queue_size = 3)
        self.return_pub = rospy.Publisher('/arduino/return_msg', UInt8Array, queue_size = 0)
        self.msg_out = UInt8Array()
        self.serial_available = True

        self.init_port(self.port,self.baudrate,timeout)

        rospy.on_shutdown(self.close_port)
        rospy.spin()

    def msg_callback(self, bytearr):
        if self.ser:
            msg = bytearray(bytearr.data[:-1])
            incoming_msgsize = struct.unpack('B', bytearr.data[-1])[0]
            while not self.serial_available:
                pass
            self.serial_available = False
            data = self.send_receive_arduino(msg, incoming_msgsize)
            self.serial_available = True
            if (len(data) > 0):
                self.msg_out.data = tuple(bytearray(struct.pack('B', msg[0]) + data))
                self.return_pub.publish(self.msg_out)

    def send_receive_arduino(self,packet,incoming_msgsize):
        checksum = sum(packet) & 0xFF
        checksum_received = None
        resend_count = 0

        while checksum_received != checksum:
            # Simple error handling only used if repeating loop
            if (checksum_received):
                if (resend_count < 1e2):
                    resend_count += 1
                    time.sleep(0.001)
                else:
                    print('Message Timeout to Arduino')
                    raise RuntimeError

            # write outgoing_packet
            self.ser.write(packet)

            # Read data packet off of serial line, we know how large this data should be..
            data = self.ser.read(incoming_msgsize)
             
            if len(data) < incoming_msgsize:
                checksum_received = None
                time.sleep(0.001)
            else:
                checksum_received = struct.unpack('B', data[0])[0]
        try:
            return data[1:]
        except:
            return

    def init_port(self, port, baudrate, timeout = 0.25):
        if self.ser:
            self.ser.close()
        else:
            self.ser = serial.Serial(port = port, baudrate = baudrate, timeout = timeout)
            time.sleep(.5)
            count = 0
            while not self.ser.isOpen():
                count += 1
                if count > 1e5:
                    raise RuntimeError
            self.ser.flushInput()
            self.ser.flushInput()
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.flushOutput()
            self.ser.flushOutput()

    def close_port(self):
        if self.ser:
            self.ser.close()

if ( __name__ == "__main__" ):
    arduino = ArduinoController()

