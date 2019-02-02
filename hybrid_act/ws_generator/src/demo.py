#! /usr/bin/env python
import wx

import rospy
from ws_generator.msg import WSArray
from std_msgs.msg import String, Bool

#Other GUI utilites
import main
import utils.demo_utils as demo_utils


class Frame(demo_utils.DemoFrame):
    #----------------------------------------------------------------------
    def __init__(self):
        """"""
        rospy.init_node('demo_ws')
        self.ws_hybrid_pub = rospy.Publisher('/cursor_position/workspace/hybrid', WSArray, queue_size = 0)
        self.master_force_pub = rospy.Publisher('/hue_master/force', Bool, queue_size = 0)
        self.master_actuation_pub = rospy.Publisher('/hue_master/actuation', Bool, queue_size = 0)

        b = Bool()
        b.data = True
        self.master_force_pub.publish(b)
        self.master_actuation_pub.publish(b)

        self.REFRESH_RATE = 15
        self.SCREEN_LENGTH = 15
        self.BALL_VELOCITY = 10     #cm/s
        demo_utils.DemoFrame.__init__(self, self.BALL_VELOCITY, self.REFRESH_RATE, self.SCREEN_LENGTH)

        # Generate Gui
        self.Centre()
        self.Show()

    def close(self):
        self.Close()
        f = main.frameMain(None)
        f.show()

# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = Frame()
    frame.Show()
    app.MainLoop()
