#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int8
from haptic_generator.msg import IntArray, WSArray

import sys
sys.path.append('../../output_controller/src/')
import Arduino

class haptic_controller(Arduino.ArduinoController):

    def __init__(self):
        rospy.init_node('haptic_control')
        self.haptic_name = rospy.get_param('~name')
        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.actuation_callback, queue_size = 1)
        self.ws_sub = rospy.Subscriber('/cursor_position/workspace/'+self.haptic_name, WSArray, self.ws_callback, queue_size = 1)
        self.int_pub = rospy.Publisher('/'+self.haptic_name+'_intensity/', Int8, queue_size = 1)

        self.last_intensity = 0
        self.ws = None

        rospy.on_shutdown(self.close)
        rospy.spin()

    def actuation_callback(self, ir_xy):
        ir_y = ir_xy.data[1]
        if self.ws:
            intensity = 0.0
            for i in range(self.ws.ystep):
                if list(self.ws.y_ws)[i*2] <= ir_y <= list(self.ws.y_ws)[i*2+1]:
                    ir_x = ir_xy.data[0]
                    intensity = list(self.ws.intensity)[ir_x]
                    break

                else:
                    intensity = 0

            if (intensity >= self.last_intensity*0.99 or intensity <= self.last_intensity*1.01):
                msg = Int8()
                msg.data = intensity

                self.int_pub.publish(msg)
                self.last_intensity = intensity

    def ws_callback(self, ws):
        self.ws = ws

    def close(self):
        self.ws = None

if __name__ == '__main__':
    controller = haptic_controller()

