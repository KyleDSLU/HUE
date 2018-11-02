#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Int8
from output_controller.msg import IntArray, ForceArray, WSArray
from Arduino import ArduinoController

class Force_Controller(ArduinoController):

    def __init__(self):
        rospy.init_node('force_control')

        self.force_case = rospy.get_param('~arduino_case')

        ArduinoController.__init__(self,port='/dev/ttyAMA0',baudrate = 57600)

        self.norm_force_publish = [[None], [None]]
        self.tan_force_publish = [[None], [None]]
        self.intensity_publish = [[None], [None]]

        self.initialize_length = 1000
        self.norm_force = [None]*self.initialize_length
        self.tan_force = [None]*self.initialize_length
        self.intensity = [None]*self.initialize_length
        self.rectangle_choice = None
        self.count = 0

        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.cursor_callback, queue_size = 1)
        self.ev_sub = rospy.Subscriber('/ev/intensity', Int8, self.ev_callback, queue_size = 1)
        self.ufm_sub = rospy.Subscriber('/ufm/intensity', Int8, self.ev_callback, queue_size = 1)
        self.ws_sub = rospy.Subscriber('/cursor_position/workspace', WSArray, self.ws_callback, queue_size = 1)
        self.master_force_sub = rospy.Subscriber('/hue_master/force', Bool, self.forcestatus_callback, queue_size = 1)
        self.force_pub = rospy.Publisher('/cursor_position/force/force_list', ForceArray, queue_size = 1)

        self.initialize()

        rospy.on_shutdown(self.close)
        rospy.spin()

    def initialize(self):
        force_return = self.send_receive(self.force_case)
        b = [0]*int(len(force_return)-1)
        forces = [0]*len(b)/2
        for i in range(len(forces)):
            forces[i] = struct.unpack('>H',x[2*i+1,2*i+3])[0]
        self.initial_forces = forces

    def ev_callback(self,intensity):
        self.ev_int = intensity.data

    def ufm_callback(self,intensity):
        self.ufm_int = intensity.data
        
    def cursor_callback(self,ir_xy):
        if self.force_status:
            if self.rectangle_choice == None:
                ir_y = ir_xy.data[1]
                for i in range(self.ystep):
                    if list(self.y_ws[i*2]) <= ir_y <= list(self.y_ws[i*2+1]):
                        self.rectangle_choice = i
                        break
           
            force_return = self.send_receive(self.force_case)
            byt = [0]*int(len(force_return)-1)
            forces = [0]*len(b)/2
            for i in range(len(forces)):
                forces[i] = struct.unpack('>H',x[2*i+1,2*i+3])[0]
            forces -= self.initial_forces
            self.norm_force[self.count] = (sum(forces[:3])/float(len(forces[:3])))
            self.tan_force[self.count] = forces[4]
            self.intensity[self.count] = self.ev_int - self.ufm_int
            self.count += 1

    def forcestatus_callback(self, force_status):
        force_status = force_status.data
        if self.force_status != force_status:
            if self.force_status:
                self.norm_force_publish[self.rectangle_choice] = self.norm_force[:self.count]
                self.tan_force_publish[self.rectangle_choice] = self.tan_force[:self.count]
                self.intensity_publish[self.rectangle_choice] = self.intensity[:self.count]
                if not (None in self.norm_force_publish):
                    force = ForceArray()
                    force.tan_force1 = self.tan_force_publish[0]
                    force.tan_force2 = self.tan_force_publish[1]
                    force.norm_force1 = self.norm_force_publish[0]
                    force.norm_force2 = self.norm_force_publish[1]
                    force.intensity_1 = self.intensity_publish[0]
                    force.intensity_2 = self.intensity_publish[1]

                    self.force_pub.Publish(force)

            self.force_status = force_status
            self.norm_force = [None]*self.initialize_length
            self.tan_force = [None]*self.initialize_length
            self.intensity = [None]*self.initialize_length
            self.rectangle_choice = None
            self.count = 0

    def ws_callback(self, ws):
        self.ystep = ws.ystep
        self.y_ws = ws.y_ws

    def close(self):
        self.close_port()

if __name__ == '__main__':
    controller = Force_Controller()
