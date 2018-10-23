#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
from ir_correction.msg import IntArray

import os

#if "SSH_CONNECTION" not in os.environ:
    #print "Ir corrector running"
#    import pyautogui
#    RUN_FLAG = 1

import pyautogui
pyautogui.FAILSAFE = False
RUN_FLAG = 1

#else:
#    RUN_FLAG = 0

class IR_Controller():

    def __init__(self, *args, **kargs):
        rospy.init_node('cursor_control')
        self.cursor_pub = rospy.Publisher('/cursor_position/corrected', IntArray, queue_size=1)
        self.raw_pub = rospy.Publisher('/cursor_position/raw', IntArray, queue_size=1)
        self.rate = 5

        self._xscale = 1.08
        self._yscale = 1.3115
        self._xoffset = -5
        self._yoffset = 0

        self._xmax,self._ymax = pyautogui.size()

        self.last_position = self.raw_position = list(pyautogui.position())

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.callback()
            r.sleep

    def callback(self):
        self.raw_position = list(pyautogui.position())

        if (self.raw_position != self.last_position):
            self.cursor_correction()

    def cursor_correction(self):
        self.raw_position_msg = IntArray()
        self.raw_position_msg.header.stamp = rospy.Time(0,0)
        self.raw_position_msg.data = self.raw_position
        self.raw_pub.publish(self.raw_position_msg)

        self.corrected_position = [int(self._xscale*(self.raw_position[0]-self._xmax/2)+self._xmax/2),int(self._yscale*(self.raw_position[1]-self._ymax/2)+self._ymax/2)]

        self.corrected_position_msg = IntArray()
        self.corrected_position_msg.header.stamp = rospy.Time(0,0)
        self.corrected_position_msg.data = self.raw_position

        self.cursor_pub.publish(self.corrected_position_msg)

        #pyautogui.moveTo(self.corrected_position[0], self.corrected_position[1])

        self.last_position = self.corrected_position


if __name__ == '__main__':
    try:
        if RUN_FLAG:
            IR_Controller()

    except rospy.ROSInterruptException:
        pass
