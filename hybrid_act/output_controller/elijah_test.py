#! /usr/bin/env python

################################################################################
# Kyle DeProw
# 9-13-2018
# Python Script to enable closed loop control of motor using an Arduino
################################################################################

import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import struct

# Enter Serial Port name in Port Section
#  Windows will have the form of 'COMXX' while Mac and Linux
#  Have the form of '/tty/someportname'
#ser = serial.Serial(port = 'COM11', baudrate = 115200, timeout = 1)

#ser = serial.Serial(port = '/dev/cu.usbmodem1411',
ser = serial.Serial(port = '/dev/cu.usbserial-AL00YX3T',baudrate = 115200, timeout = 1)



# Delay a bit for Serial Port to open/configure (Windows slows this down)
time.sleep(2)
count = 0
while not ser.isOpen():
    count += 1
    if count > 1e5:
        raise RuntimeError

##################################################################################
# Functions

# Function to handle sending bytes to Arduino.  Returns Arduino's response in bytes.'
#   packet should be a byte array of the outgoing_packet
def send_receive_arduino(packet, incoming_msgsize):

    checksum = sum(packet) & 0xFF
    print(packet,len(packet))
    checksum_received = None
    resend_count = 0
    while  checksum_received != checksum:
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
            print('Data read in is shorter than expected message size...  '+ 'packet:'+ packet + 'data:'+ data )
            #print(checksum)
            checksum_received = None
        else:
            dummy  = data[0]
            checksum_received = struct.unpack('B', dummy)[0]
    print(packet,checksum,checksum_received,data)
    return data

##################################################################################
# Main Script

in1 = 0;
in2 = 0;
in3 = 0;
in4 = 0;
in5 = 0;


incoming_msgsize = 1
reset_char = 'r'
reset_size = 1      # Only expect 1 byte back from Arduino during reset
speed_char = '4'
speed_size = 3      # Expect 3 bytes back from Arduino during speed comma-40
percentage_of_speed = 5000
motor_speed_bytes = struct.pack('>h',int(percentage_of_speed/100.*255))
time_to_sleep = 1

#Send Motor Speed
outgoing_packet = bytearray(speed_char) + motor_speed_bytes
print(outgoing_packet,len(outgoing_packet))

if(speed_char == '2'):
        starting_count = send_receive_arduino(outgoing_packet,1)

elif(speed_char == '3'):
        starting_count = send_receive_arduino(outgoing_packet,1)

elif(speed_char == '4'):
        starting_count = send_receive_arduino(outgoing_packet,11)
        dummy = starting_count[1]
        in1 = struct.unpack('B', dummy)[0];
        dummy = starting_count[2]
        in2 = struct.unpack('B', dummy)[0];
        dummy = starting_count[3]
        in1 = struct.unpack('B', dummy)[0];
        dummy = starting_count[4]
        in1 = struct.unpack('B', dummy)[0];
        dummy = starting_count[5]
        in1 = struct.unpack('B', dummy)[0];
        print in1
        print in2
        print in3
        print in4
        print in5
# Deal with Arduino's reply if necessary
#motor_counts = struct.unpack('>h',ending_count[1:])[0] - struct.unpack('>h',starting_count[1:])[0]
#print motor_counts
ser.close()
