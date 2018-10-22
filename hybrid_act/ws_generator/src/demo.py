#! /usr/bin/env python
import wx
import os
# import rospy

import random
import time

# from ws_generator.msg import WSArray
# from std_msgs.msg import String

#Other GUI utilites
import main
import demo_utils


class Frame(demo_utils.DemoFrame):
    #----------------------------------------------------------------------
    def __init__(self,csvfile):
        """"""
        # self.ws_ufm_pub = rospy.Publisher('/cursor_position/workspace/ufm', WSArray, queue_size = 0)
        # self.ws_ev_pub = rospy.Publisher('/cursor_position/workspace/ev', WSArray, queue_size = 0)
        # rospy.init_node('start_ws')

        self.REFRESH_RATE = 20
        self.SCREEN_LENGTH = 15
        self.BALL_VELOCITY = 10     #cm/s
        demo_utils.DemoFrame.__init__(self, self.BALL_VELOCITY, self.REFRESH_RATE, self.SCREEN_LENGTH)

        # Generate Gui
        self.Centre()
        self.Show()
        # Generate ws
        # self.generate_ws()

# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = Frame('./csvfiles/test.py')
    frame.Show()
    app.MainLoop()
