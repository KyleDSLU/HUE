#!/usr/bin/env python

import struct
import time
import os
import rospkg
import rospy 
from std_msgs.msg import Bool, Float32, Int16, UInt16MultiArray, Int8
from output_controller.msg import ForceArray, ForceChannel, WSArray, IntArray, UInt8Array
from MAX518 import MAX518_Controller

import numpy as np

class Force_Controller(MAX518_Controller):

    def __init__(self):
        rospy.init_node('force_control')
        self.force_case = struct.pack('B', rospy.get_param('~force_case'))
        self.phase_case = struct.pack('B', rospy.get_param('~phase_case'))
        self.freq_case = struct.pack('B', rospy.get_param('~freq_case'))
        self.version_case = struct.pack('B', rospy.get_param('~version_case'))
        self.force_msg_return = struct.pack('B', rospy.get_param('~force_msg_length'))
        self.haptic_name = rospy.get_param('~name')

        # Initialize subs and pubs
        self.force_pub = rospy.Publisher('/force_recording/force_records', \
                                         ForceArray, queue_size=0)

        self.msg_pub = rospy.Publisher('/arduino/msgout', \
                                       UInt8Array, queue_size=1)

        self.normforce_pub = rospy.Publisher('/force_recording/normal_force', \
                                             Float32, queue_size=0)
        self.arduino_lockout_pub = rospy.Publisher('/arduino/lockout', \
                                                   Bool, queue_size=0)

        # Initialize constants
        self.INITIALIZE_LENGTH = 200
        self.COUNT_THRESHOLD = self.INITIALIZE_LENGTH/2

        # Clear Functional members
        self.msg = UInt8Array()
        self.norm_force_scaled = Float32()
        self.count = 0
        self.channels = 0
        self.forcechan = None
        self.intensity_latest = None
        self.initial_forces = None
        self.force_status = False
        self.forces = None
        self.lengths = [0]*self.channels
        self.norm_force_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
        self.tan_force_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
        self.x_position_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
        self.intensity_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
        self.ufm_amp_controller = None
        self.ev_amp_controller = None

        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', \
                                        IntArray, self.cursor_callback, queue_size=1)

        self.int_sub = rospy.Subscriber('/'+self.haptic_name+'/intensity/', \
                                        Int16, self.int_callback, queue_size=1)

        self.ws_sub = rospy.Subscriber('/cursor_position/workspace', \
                                       WSArray, self.ws_callback, queue_size=1)

        self.force_status_sub = rospy.Subscriber('/hue_master/force', \
                                                 Bool, self.forcestatus_callback, queue_size=1)

        self.forcechan_sub = rospy.Subscriber('/force_recording/force_channel', \
                                              ForceChannel, self.forcechan_callback, queue_size=1)

        self.msg_return_sub = rospy.Subscriber('/arduino/return_msg', \
                                               UInt8Array, self.msg_callback, queue_size=1)

        self.version_sub = rospy.Subscriber('/hue_master/version', \
                                            Int8, self.version_callback, queue_size=1)


        self.freq_sub = rospy.Subscriber('/'+self.haptic_name+'/freqs', \
                                         UInt16MultiArray, self.freq_callback, queue_size=1)

        time.sleep(2)

        #FSA 10N sensor with 10-90% Transfer function in 5V output
        self.FORCE_SENSOR_OFFSET_COUNTS = 0.1*1024
        self.FORCE_SENSOR_SCALING = 10/(1024.-2*self.FORCE_SENSOR_OFFSET_COUNTS)

        # Initialize i2c controllers
        self.ufm_amp_controller = MAX518_Controller(rospy.get_param('~ufm_i2c_address'))
        self.ev_amp_controller = MAX518_Controller(rospy.get_param('~ev_i2c_address'))
        self.ufm_A0max = 4.1 # *rospy.get_param('~ufm_scale')
        self.ufm_A1max = .95 # *rospy.get_param('~ufm_scale')

        # EV voltage calculations for V2
        self.v_t = 4.75
        self.v_dc = self.v_t/(1.3)
        self.v_ac = 0.6*self.v_dc

        # Turn off Arduino Lockout
        self.arduino_lockout_pub.publish(Bool(False))

        # Initialize force arrays and x_position
        self.nan = np.empty(self.INITIALIZE_LENGTH)
        self.nan[:] = np.nan
        self.norm_force = np.zeros_like(self.nan)
        self.tan_force = np.zeros_like(self.nan)
        self.x_position = np.zeros_like(self.nan)
        self.intensity = np.zeros_like(self.nan)

        self.norm_force[:] = self.nan[:]
        self.tan_force[:] = self.nan[:]
        self.x_position[:] = self.nan[:]
        self.intensity[:] = self.nan[:]

        # Initialize Force Collection members
        self.msg.data = tuple(bytearray(self.force_case + self.force_msg_return))
        self.msg_pub.publish(self.msg)
        self.lengths = [0]
        self.norm_force_publish = np.zeros((1, self.INITIALIZE_LENGTH))
        self.tan_force_publish = np.zeros((1, self.INITIALIZE_LENGTH))
        self.x_position_publish = np.zeros((1, self.INITIALIZE_LENGTH))
        self.intensity_publish = np.zeros((1, self.INITIALIZE_LENGTH))
        self.norm_force_publish[0][:] = self.nan[:]
        self.tan_force_publish[0][:] = self.nan[:]
        self.x_position_publish[0][:] = self.nan[:]
        self.intensity_publish[0][:] = self.nan[:]

        # default to HUE version 1
        self.hue_version = 1
        self.ufm_freq = 21000
        self.ev_freq = 30800
        output = UInt16MultiArray()
        output.data = [self.ufm_freq, self.ev_freq]
        self.freq_callback(output)
        #self.version_callback(Int8(self.hue_version))

        rospy.spin()

    def version_callback(self, version):
        self.hue_version = version.data

        # read voltage solutions from csv
        rospack = rospkg.RosPack()
        path = rospack.get_path("output_controller")
        dir_path = os.path.join(path, 'src/csv/ev_hue_v' + str(self.hue_version) + '/')

        f_solutions = os.path.join(dir_path, 'solutions.csv')
        f_key = os.path.join(dir_path, 'key.csv')
        f_amp = os.path.join(dir_path, 'amp.csv')

        self.solutions_read = np.genfromtxt(f_solutions, delimiter=',')
        self.key_read = np.genfromtxt(f_key, delimiter=',')
        self.amp_read = np.genfromtxt(f_amp, delimiter=',')
        # reshape solutions array
        self.solutions_read.shape = tuple(self.key_read.astype(int))

        if self.hue_version == 1:
            self.ufm_amp_controller.DAC_output(0, 0)
            self.ev_amp_controller.DAC_output(0, 0)
            self.msg.data = tuple(bytearray(self.version_case + \
                                            struct.pack("B", self.hue_version) + \
                                            struct.pack('>H', self.ufm_freq) + \
                                            struct.pack('>H', self.ev_freq) + \
                                            b'\x01'))
            self.msg_pub.publish(self.msg)
            # lockout Arduino Comm when switching version
            self.arduino_lockout_pub.publish(Bool(True))
            time.sleep(1.0)
            self.arduino_lockout_pub.publish(Bool(False))

        elif self.hue_version == 2:
            self.msg.data = tuple(bytearray(self.version_case + \
                                            struct.pack("B", self.hue_version) + \
                                            struct.pack('>H', self.ufm_freq) + \
                                            struct.pack('>H', self.ev_freq) + \
                                            b'\x01'))
            self.msg_pub.publish(self.msg)
            # lockout Arduino Comm when switching version
            self.arduino_lockout_pub.publish(Bool(True))

            self.ufm_amp_controller.DAC_output(self.ufm_A0max, self.ufm_A1max)
            outputs = self.interpolate(self.v_ac)
            self.ev_amp_controller.DAC_output(outputs[0], outputs[1])
            time.sleep(1.0)
            self.arduino_lockout_pub.publish(Bool(False))

    def forcechan_callback(self, forcechannel):
        self.channels = forcechannel.channels
        self.forcechan = forcechannel.measure_channel
        # Extend arrays if more than 1 channel
        if self.channels != self.norm_force_publish.shape[0]:
            self.lengths = [0]*self.channels
            self.norm_force_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
            self.tan_force_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
            self.x_position_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))
            self.intensity_publish = np.zeros((self.channels, self.INITIALIZE_LENGTH))

            for i in range(self.channels):
                self.norm_force_publish[i][:] = self.nan[:]
                self.tan_force_publish[i][:] = self.nan[:]
                self.x_position_publish[i][:] = self.nan[:]
                self.intensity_publish[i][:] = self.nan[:]

    def int_callback(self, intensity):
        if self.hue_version == 1:
            if intensity.data > 0:
                output_amp = (intensity.data/1000.)*4.8
                outputs = self.interpolate(output_amp)
                self.ufm_amp_controller.DAC_output(0, 0)
                self.ev_amp_controller.DAC_output(outputs[0], outputs[1])
            elif intensity.data < 0:
                self.ufm_amp_controller.DAC_output(abs(intensity.data/1000.)*self.ufm_A0max, \
                                                   abs(intensity.data/1000.)*self.ufm_A1max)
                self.ev_amp_controller.DAC_output(0, 0)
            else:
                self.ufm_amp_controller.DAC_output(0, 0)
                self.ev_amp_controller.DAC_output(0, 0)

        if self.hue_version == 2:
            # AD9833 reads phase with decimal shifted eg. 110.1 degrees = 1101
            intensity_to_phase = int(intensity.data/1000. * 90 + 90) * 10
            self.msg.data = tuple(bytearray(self.phase_case + \
                                  struct.pack('>H', intensity_to_phase) + \
                                  b'\x01'))
            self.msg_pub.publish(self.msg)

        self.intensity_latest = intensity
        if self.force_status:
            self.msg.data = tuple(bytearray(self.force_case + self.force_msg_return))
            self.msg_pub.publish(self.msg)

    def msg_callback(self, bytearr):
        case = bytearr.data[0]
        if case == self.force_case:
            forces = self.force_unpack(bytearr.data[1:])
            if self.initial_forces is None:
                self.initial_forces = forces
            else:
                self.forces = forces# - self.initial_forces

    def cursor_callback(self,ir_xy):
        if self.force_status and \
           self.intensity_latest is not None and \
           self.forces != None and \
           self.count < self.INITIALIZE_LENGTH:

            # Offset for initial conditions
            forces = self.forces
            norm_force_unscaled = int(sum(forces[:2])/float(len(forces[:2])))
            self.norm_force[self.count] = norm_force_unscaled

            self.norm_force_scaled.data = 4*(norm_force_unscaled-self.FORCE_SENSOR_OFFSET_COUNTS)*\
                                             self.FORCE_SENSOR_SCALING
            self.normforce_pub.publish(self.norm_force_scaled)

            self.tan_force[self.count] = forces[3]
            self.x_position[self.count] = ir_xy.data[0]
            self.intensity[self.count] = self.intensity_latest
            self.count += 1

    def forcestatus_callback(self, force_status):
        force_status = force_status.data
        if self.force_status != force_status:
            if self.force_status and self.count >= self.COUNT_THRESHOLD:
                self.norm_force_publish[self.forcechan][:self.count-1] = self.norm_force[:self.count-1]
                self.tan_force_publish[self.forcechan][:self.count-1] = self.tan_force[:self.count-1]
                self.x_position_publish[self.forcechan][:self.count-1] = self.x_position[:self.count-1]
                self.intensity_publish[self.forcechan][:self.count-1] = self.intensity[:self.count-1]
                self.lengths[self.forcechan] = self.count

                norm_force_publish = self.norm_force_publish.flatten()
                norm_force_publish = norm_force_publish[~np.isnan(norm_force_publish)]

                tan_force_publish = self.tan_force_publish.flatten()
                tan_force_publish = tan_force_publish[~np.isnan(tan_force_publish)]

                x_position_publish = self.x_position_publish.flatten()
                x_position_publish = x_position_publish[~np.isnan(x_position_publish)]

                intensity_publish = self.intensity_publish.flatten()
                intensity_publish = intensity_publish[~np.isnan(intensity_publish)]

                force = ForceArray()
                force.channels = self.channels
                force.lengths = self.lengths
                force.tan_force = tan_force_publish.flatten().tolist()
                force.norm_force = norm_force_publish.flatten().tolist()
                force.x_positions = x_position_publish.flatten().tolist()
                force.intensity = intensity_publish.flatten().tolist()
                force.channels = self.channels
                self.force_pub.publish(force)

            self.force_status = force_status
            self.norm_force[:] = self.nan
            self.tan_force[:] = self.nan
            self.x_position[:] = self.nan
            self.intensity_latest = None
            self.count = 0

    def force_unpack(self, force_input):
        force_input = bytearray(force_input)
        forces = np.zeros(len(force_input)/2)
        for i in enumerate(forces):
            forces[i[0]] = struct.unpack('>H', force_input[2*i[0]:2*i[0]+2])[0]
        return forces

    def interpolate(self, amp_desired):
        ind = np.searchsorted(self.amp_read, amp_desired, 'right')
        if self.amp_read[ind-1] == amp_desired:
            solution = np.zeros_like(self.solutions_read[ind-1])
            solution[:] = self.solutions_read[ind-1]

        else:
            solution = np.zeros_like(self.solutions_read[ind])
            spacing = self.amp_read[ind] - self.amp_read[ind-1]
            diff = amp_desired - self.amp_read[ind-1]
            for i in enumerate(solution):
                ends = [self.solutions_read[:, i[0]][ind-1], \
                        self.solutions_read[:, i[0]][ind]]
                solution[i[0]] = np.interp(diff, np.linspace(0,spacing,len(ends)), ends)
        return solution

    def ws_callback(self, ws):
        self.ystep = ws.ystep
        self.y_ws = ws.y_ws

    def freq_callback(self, freq):
        self.ufm_freq, self.ev_freq = freq.data[0], freq.data[1]
        self.version_callback(Int8(self.hue_version))

if __name__ == '__main__':
    controller = Force_Controller()
