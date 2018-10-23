#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8, UInt16
from output_controller.msg import IntArray
from MAX518 import MAX518_Controller
from Arduino import ArduinoController

class Output_Controller(ArduinoController,MAX518_Controller):

    def __init__(self):
        rospy.init_node('output_control')

        try:
            self.haptic_name = rospy.get_param('~name')
            i2c_address = rospy.get_param('~i2c_address')
            self.arduino_case = rospy.get_param('~arduino_case')
            scale = rospy.get_param('~scale')
        except:
            # run this is not executed through launch file
            self.haptic_name = 'ev'
            i2c_address = 44
            self.arduino_case = 3
            scale = 0.9

        MAX518_Controller.__init__(self,i2c_address)
        ArduinoController.__init__(self,port='/dev/ttyARDUINO',baudrate = 57600)

        self._A0max = 4.1*scale
        self._A1max = 1.05*scale

        self.int_sub = rospy.Subscriber('/'+self.haptic_name+'/intensity/', Int8, self.int_callback, queue_size = 1)
        self.freq_sub = rospy.Subscriber('/'+self.haptic_name+'/frequency/', UInt16, self.freq_callback, queue_size = 1)

        rospy.on_shutdown(self.close)
        rospy.spin()

    def int_callback(self, intensity):
        intensity = intensity.data
        if self._i2cbus:
            self.DAC_output(self._A0max*intensity/100., self._A1max*intensity/100.)

    def freq_callback(self, frequency):
#        print(frequency.data)
        frequency = frequency.data
        if self.ser:
            self.send_receive(self.arduino_case,frequency,'>H')

    def close(self):
        self.MAX518_close()
        self.close_port()

if __name__ == '__main__':
    controller = Output_Controller()

