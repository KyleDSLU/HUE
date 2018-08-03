#!/usr/bin/env python

import rospy
import time
import struct
from std_msgs.msg import UInt16
from Arduino import ArduinoController

class FrequencyController(ArduinoController):

    def __init__(self):
        rospy.init_node('output_control')
        
        self.haptic_name = rospy.get_param('~name')
        self.arduino_case = rospy.get_param('~arduino_case')
        ArduinoController.__init__(self,port='/dev/ttyACM0',baudrate = 115200)

        self.freq_sub = rospy.Subscriber('/'+self.haptic_name+'/frequency/', UInt16, self.freq_callback, queue_size = 1)

        rospy.on_shutdown(self.close)
        rospy.spin()

    def freq_callback(self, frequency):
        frequency = frequency.data
        if self.ser:
            self.send_receive(self.arduino_case,frequency)

    def close(self):
        self.closePort()

if __name__ == '__main__':
    controller = FrequencyController()
    
