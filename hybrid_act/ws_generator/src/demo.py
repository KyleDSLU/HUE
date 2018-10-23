#! /usr/bin/env python
import wx

#Other GUI utilites
from ws_generator.msg import WSArray
import main
import demo_utils


class Frame(demo_utils.DemoFrame):
    #----------------------------------------------------------------------
    def __init__(self,csvfile):
        """"""
        rospy.init_node('demo_ws')
        self.ws_ufm_pub = rospy.Publisher('/cursor_position/workspace/ufm', WSArray, queue_size = 0)
        self.ws_ev_pub = rospy.Publisher('/cursor_position/workspace/ev', WSArray, queue_size = 0)
        self.master_force_pub = rospy.Publisher('/hue_master/force', Bool, queue_size = 0)
        self.master_actuation_pub = rospy.Publisher('/hue_master/actuation', Bool, queue_size = 0)

        b = Bool
        b.data = True
        self.master_force_pub(b)
        self.master_actuation_pub(b)

        self.REFRESH_RATE = 20
        self.SCREEN_LENGTH = 15
        self.BALL_VELOCITY = 10     #cm/s
        demo_utils.DemoFrame.__init__(self, self.BALL_VELOCITY, self.REFRESH_RATE, self.SCREEN_LENGTH)

        # Generate Gui
        self.Centre()
        self.Show()

# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = Frame('./csvfiles/test.py')
    frame.Show()
    app.MainLoop()
