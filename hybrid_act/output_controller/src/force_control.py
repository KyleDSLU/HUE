#!/usr/bin/env python

import rospy
from output_controller import IntArray
from Arduino import ArduinoController

class Force_Controller(ArduinoController):

    def __init__(self):
        rospy.init_node('force_control')

        self.arduino_case = rospy.get_param('~arduino_case')

        ArduinoController.__init__(self,port='/dev/ttyARDUINO',baudrate = 57600)

        self.force_list = 0
        self.x_position = 0

        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.cursor_callback, queue_size = 1)
        self.force_pub = rospy.Publisher('/cursor_position/force/force_list', IntArray, queue_size = 1)
        self.x_pub = rospy.Publisher('/cursor_position/force/x_position', IntArray, queue_size = 1)

        rospy.on_shutdown(self.close)
        rospy.spin()

    def cursor_callback(self,cursor)
        if self.force_status:
            ir_x = ir_xy.data[0]
            force_return = self.send_receive(self.force_case)
            x = [0]*int(len(force_return))
            for i in range(len(y)):
                y[i] = struct.unpack('H',x[2*i,2*i+2])[0]
            self.force_list.append(force_return)
            self.x_position.append(ir_x)
        else:
            self.force_list = 0
            self.x_position = 0

    def force_callback(self, force_status):
        force_status = force_status.data
        if self.force_status != force_status:
            if self.force_status:
                force = IntArray()
                force.data = self.force_list
                x = IntArray()
                x.data = self.x_position

                self.force_pub.Publish(force)
                self.x_pub.Publish(x)

            self.force_status = force_status

    def close(self):
        self.close_port()

if __name__ == '__main__':
    controller = Force_Controller()
