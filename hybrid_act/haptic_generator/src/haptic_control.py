#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import Int16, Bool
from haptic_generator.msg import IntArray, WSArray

class haptic_controller():

    def __init__(self):
        self.ws = None

        self.master = False

        rospy.init_node('haptic_control')
        self.haptic_name = rospy.get_param('~name')
        self.master_sub = rospy.Subscriber('/hue_master/actuation', Bool, self.master_callback, queue_size = 1)
        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.actuation_callback, queue_size = 1)
        self.ws_sub = rospy.Subscriber('/cursor_position/workspace/'+self.haptic_name, WSArray, self.ws_callback, queue_size = 1)
        self.int_pub = rospy.Publisher('/'+self.haptic_name+'/intensity/', Int16, queue_size = 0)

        msg = Int16()
        msg.data = 0
        self.int_pub.publish(msg)
        self.last_intensity = 0

        self.threshold = 1

        rospy.on_shutdown(self.close)
        rospy.spin()

    def actuation_callback(self, ir_xy):
        ir_y = ir_xy.data[1]
        if self.ws and self.master:
            intensity = 0
            for i in range(self.ws.y_step):
                if list(self.ws.y_ws)[i*2] <= ir_y <= list(self.ws.y_ws)[i*2+1]:
                    # scale x position by ws compression constant
                    ir_x = int(ir_xy.data[0]/self.ws.int_compress)
                    intensity = list(self.ws.intensity)[i*self.division+ir_x]
                    break

            if (intensity >= self.last_intensity+self.threshold or intensity <= self.last_intensity-self.threshold):
                msg = Int16()
                msg.data = intensity
                self.int_pub.publish(msg)
                self.last_intensity = intensity

        else:
            intensity = 0
            if (intensity >= self.last_intensity+self.threshold or intensity <= self.last_intensity-self.threshold):
                msg = Int16()
                msg.data = intensity
                self.int_pub.publish(msg)
                self.last_intensity = intensity

    def ws_callback(self, ws):
        self.ws = ws
        self.division = len(ws.intensity)/2

    def master_callback(self, master):
        self.master = master.data

    def close(self):
        self.ws = None

if __name__ == '__main__':
    controller = haptic_controller()

