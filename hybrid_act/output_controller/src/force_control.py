#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32
from output_controller.msg import ForceArray, ForceChannel, WSArray
from Arduino import ArduinoController

class Force_Controller(ArduinoController):

    def __init__(self):
        rospy.init_node('force_control')

        self.force_case = rospy.get_param('~arduino_case')

        ArduinoController.__init__(self, port='/dev/ttyAMA0', baudrate = 115200)

        # Initialize constants
        self.INITIALIZE_LENGTH = 200
        self.COUNT_THRESHOLD = 100

        # Initialize subs and pubs
        self.ws_sub = rospy.Subscriber('/cursor_position/workspace', WSArray, self.ws_callback, queue_size = 1)
        self.force_status_sub = rospy.Subscriber('/hue_master/force', Bool, self.forcestatus_callback, queue_size = 1)
        self.forcechan_sub = rospy.Subscriber('/force_recording/force_channel', ForceChannel, self.forcechan_callback, queue_size = 1)
        self.force_pub = rospy.Publisher('/force_recording/force_records', ForceArray, queue_size = 1)
        self.normforce_pub = rospy.Publisher('/force_recording/normal_force', Float32, queue_size = 1)
        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.cursor_callback, queue_size = 1)

        # Initialize force arrays and x_position
        self.nan = np.empty([1,self.INITIALIZE_LENGTH])
        self.nan[:] = np.nan
        self.norm_force[:] = self.nan[:]
        self.tan_force[:] = self.nan[:]
        self.x_position[:] = self.nan[:]
        nan_vector = np.empty(len(self.nan))
        nan_vector[:] = self.nan[:] 
        nan_vector.shape = (1,self.INITIALIZE_LENGTH)
        self.norm_force_publish[:] = nan_vector[:]
        self.tan_force_publish[:] = nan_vector[:] 
        self.x_position_publish[:] = nan_vector[:] 

        self.count = 0
        self.forcechan = None

        self.initialize()

        rospy.on_shutdown(self.close)
        rospy.spin()

    def initialize(self):
        self.initial_forces = self.force_unpack(self.send_receive(self.force_case))

    def forcechan_callback(self, forcechannel):
        self.channels = forcechannel.channels
        self.forcechan = forcechannel.measure_channel
        # Extend arrays if more than 1 channel
        if self.channels != len(self.norm_force_publish):
            l = len(self.norm_force)
            for i in range(self.channels-l-1):
                self.norm_force_publish = np.vstack([self.norm_force, self.nan])
                self.tan_force_publish = np.vstack([self.tan_force, self.nan])
                self.x_position_publish = np.vstack([self.x_position, self.nan])

    def cursor_callback(self,ir_xy):
        if self.force_status:
            forces = self.force_unpack(self.send_receive(self.force_case))
            # Offset for initial conditions
            forces -= self.initial_forces

            self.norm_force[self.count] = int(sum(forces[:3])/float(len(forces[:3]))))
            self.tan_force[self.count] = forces[4]
            self.x_position[self.count] = ir_xy.data[0]
            self.count += 1

    def forcestatus_callback(self, force_status):
        force_status = force_status.data
        if self.force_status != force_status:
            if self.force_status == True and count >= self.COUNT_THRESHOLD:
                self.norm_force_publish[self.forcechan][:self.count] = self.norm_force[:self.count]
                self.tan_force_publish[self.forcechan][:self.count] = self.tan_force[:self.count]
                self.x_position_publish[self.forcechan][:self.count] = self.x_position[:self.count]

                force = ForceArray()
                force.tan_force = self.tan_force_publish.flatten().tolist()
                force.norm_force = self.norm_force_publish.flatten().tolist()
                force.x_position = self.intensity_publish.flatten().tolist()
                force.channels = self.channels
                self.force_pub.Publish(force)

            self.force_status = force_status
            self.norm_force[:] = self.nan
            self.tan_force[:] = self.nan
            self.x_position[:] = self.nan
            self.forcechan = None
            self.count = 0

    def force_unpack(self, force_input):
        forces = [0]*len((force_input)/2)
        for i in range(len(forces)):
            forces[i] = struct.unpack('>H',force_input[2*i,2*i+2])[0]
        return forces

    def ws_callback(self, ws):
        self.ystep = ws.ystep
        self.y_ws = ws.y_ws

    def close(self):
        self.close_port()

if __name__ == '__main__':
    controller = Force_Controller()
