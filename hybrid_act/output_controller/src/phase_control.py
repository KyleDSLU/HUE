#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, UInt16
from output_controller.msg import IntArray
from Arduino import ArduinoController

class Output_Controller(ArduinoController):

    def __init__(self):
        rospy.init_node('output_control')

        self.haptic_name = rospy.get_param('~name')
        self.freq_case = rospy.get_param('~freq_case')
        self.phase_case = rospy.get_param('~phase_case')
        scale = rospy.get_param('~scale')

        ArduinoController.__init__(self, port='/dev/ttyAMA0', baudrate = 115200)

        self.int_sub = rospy.Subscriber('/'+self.haptic_name+'/intensity/', Int8, self.int_callback, queue_size = 1)

        rospy.on_shutdown(self.close)
        rospy.spin()

    def int_callback(self, intensity):
        # Scale 0-100 scale to 0-180 degrees then shift over to account for tenth of degree
        intensity = int(intensity.data/10. * 180)
        if self.ser:
            self.send_receive(self.phase_case, intensity, '>H')

    def close(self):
        self.close_port()

if __name__ == '__main__':
    controller = Output_Controller()

