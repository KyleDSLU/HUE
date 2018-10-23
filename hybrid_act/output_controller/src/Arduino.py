#! /usr/bin.env python
import serial
import time
import struct

class ArduinoController():

    def __init__(self,port=None,baudrate=57600,timeout=.25):
        if port:
            self.init_port(port,baudrate,timeout)
        else:
            self.ser = None

    def send_receive(self,arduino_case,msg=None,encoding=None):
        if self.ser:
            packet = struct.pack('B',arduino_case)
            if msg:
                packet += struct.pack(encoding,msg)

            response = self.ser.send_receive_arduino(packet, 1)
            return response[1:]

        else:
            print ('Serial Communication not Established')

    def send_receive_arduino(self,packet,incoming_msgsize):
        checksum = sum(packet) & 0xFF
        checksum_received = None
        resend_count = 0

        while checksum_received != checksum:
            # Simple error handling only used if repeating loop
            if (checksum_received):
                if (resend_count < 1e2):
                    resend_count += 1
                    print('Checksum did not agree! Resending', checksum, checksum_received)
                else:
                    print('Message Timeout to Arduino')
                    raise RuntimeError

            # write outgoing_packet
            ser.write(packet)

            # Read data packet off of serial line, we know how large this data should be..
            data = ser.read(incoming_msgsize)
            if len(data) < incoming_msgsize:
                resend_count += 1
                print('Data read in is shorter than expected message size')
            else:
                checksum_received = struct.unpack('B',data[0])[0]
        return data

    def init_port(self, port, baudrate, timeout = 0.25):
        if self.ser:
            self.ser.close()
        else:
            self.ser = serial.Serial(port = port,baudrate = baudrate , timeout = timeaout)
            time.sleep(2)
            count = 0
            while not ser.isOpen():
                count += 1
                if count > 1e5:
                    raise RuntimeError

    def close_port(self):
        if self.ser:
            self.ser.close()

if ( __name__ == "__main__" ):
    arduino = frequency_controller(port = '/dev/ttyACM1', baudrate = 115200)

