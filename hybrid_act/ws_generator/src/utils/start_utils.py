#! /usr/bin/env python
import os
import time 
import numpy as np 
import wx

from ws_generator.msg import WSArray, IntArray, ForceChannel
from std_msgs.msg import Bool
import rospkg
import rospy
import main

from utils import Ball, Generate_WS, NormForcePanel, NormForceLabel

class DemoPanel(wx.Panel):
    def __init__(self, parent, velocity, refresh, length):
        wx.Panel.__init__(self, parent)
        self.overlay = wx.Overlay()
        self.parent = parent

        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.SetBackgroundColour("BLACK")

        self.rospack = parent.rospack
        self.last_pos = self.ScreenToClient(wx.GetMousePosition())

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.REFRESH = refresh

        #setting up init pause
        self.INIT_WAIT = 0.5        # seconds
        self.INIT_WAIT = self.INIT_WAIT/(self.REFRESH/1000.)
        self.init_pause = 0

        self.LENGTH = length
        self.BALL_VELOCITY = velocity
        self.screenDC = wx.ScreenDC()

        self.BITMAP_FLAG = False
        self.OVERRIDE = False
        self.updateFlag = False
        self.on_size(0)

        self.force_channel = [ForceChannel(), ForceChannel()]
        self.force_channel[0].channels = 2
        self.force_channel[0].measure_channel = 0
        self.force_channel[1].channels = 2
        self.force_channel[1].measure_channel = 1

        self.forcechannel_pub = rospy.Publisher('/force_recording/force_channel', ForceChannel, queue_size = 0)
        self.master_force_pub = rospy.Publisher('/hue_master/force', Bool, queue_size = 0)
        self.master_actuation_pub = rospy.Publisher('/hue_master/actuation', Bool, queue_size = 0)
        # Setup subscriber for cursor position, will run on_update for callback
        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.cursor_callback, queue_size = 1)
        
        wx.CallLater(200, self.SetFocus)

    def on_size(self, event):
        self.WIDTH, self.HEIGHT = self.GetClientSize()
        self.CURSOR_OFFSET = self.parent.HEIGHT - self.HEIGHT
        self.WIDTH_HALF = self.WIDTH/2.0
        self.FIRST_RECTANGLE_Y = 0.15*self.HEIGHT
        self.BOTTOM_SPACE = 0.035*self.HEIGHT

        self.RECTANGLE_SIZE = .25*self.HEIGHT
        self.RECTANGLE_SEPERATION = int((self.HEIGHT-self.FIRST_RECTANGLE_Y-self.BOTTOM_SPACE-self.RECTANGLE_SIZE)/3.0)

        #set up textbox and lines
        self.TEXTBOX_WIDTH = 0.065*self.WIDTH
        self.TEXTBOX_X = 0.04*self.WIDTH
        self.TEXTBOX_XOFFSET = 0.80*self.WIDTH
        self.TEXTBOX_Y = self.RECTANGLE_SIZE*0.35
        self.RECTANGLE_COLOR = "GREY"
        self.TEXTBOX_FONTSIZE = max(1,int(10*((self.WIDTH*self.HEIGHT)/(1794816.))))
        self.TEXTBOX_COLOR = "WHITE"

        self.HAPTIC_WIDTH = self.WIDTH - self.TEXTBOX_WIDTH

        self.BALL_RADIUS = int(self.RECTANGLE_SIZE/2*1.1)
        self.BALL_YOFFSET = -0.0125*self.HEIGHT
        self.BALL_LEFTMARGIN = 0.95*self.BALL_RADIUS+self.TEXTBOX_WIDTH
        self.BALL_RIGHTMARGIN = self.WIDTH-2.0*self.BALL_RADIUS
        self.WAIT = 15   # Parameter to set for ball wait before reset

        self.ball = [[],[]]
        self.wait_count = [0]*len(self.ball)
        first_ball_start = int(self.FIRST_RECTANGLE_Y+self.BALL_RADIUS+self.BALL_YOFFSET)
        second_ball_start = int(first_ball_start+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        self.BALL_START = [[self.BALL_LEFTMARGIN,first_ball_start],[self.BALL_LEFTMARGIN,second_ball_start]]#,[self.BALL_LEFTMARGIN,third_ball_start]]

        for i in range(len(self.ball)):
            self.ball[i] = Ball(self.BALL_START[i],self.BALL_RADIUS,self.BALL_RIGHTMARGIN)

        self.BALL_MOVEX = int(self.BALL_VELOCITY*self.REFRESH*self.WIDTH/(2450.*self.LENGTH))

    def update_drawing(self, *args):
        self.on_update(self.ir_x, self.ir_y)

    def cursor_callback(self, ir_xy):
        self.ir_x = ir_xy.data[0]
        self.ir_y = ir_xy.data[1] - self.CURSOR_OFFSET

    def override_on_paint(self):
        self.OVERRIDE = True
        self.on_paint()
        self.OVERRIDE = False

    def on_paint(self, *args):
        if self.updateFlag == True:
            self.updateFlag = False
            self.clearPanel()

        if self.WIDTH > 20 and self.HEIGHT > 20:
            if not self.BITMAP_FLAG:
                path = self.rospack.get_path('ws_generator')
                path = os.path.join(path, 'src/utils/ref/')
                dc = wx.PaintDC(self)
                dc.Clear()

                # Import Bitmap
                self.import_bitmap(dc, path)
                self.BITMAP_FLAG = 1

                # Draw Bitmap and setup overlay
                dc.DrawBitmap(self.bmp, 0, 0)
                self.overlay = wx.Overlay()
                self.odc = wx.DCOverlay(self.overlay, dc)
                self.odc.Clear()
                del dc

                # Draw Balls for first time
                redraw_list = [True] * len(self.ball)
                pos_list = self.BALL_START
                self.draw_balls(redraw_list, pos_list)

        elif self.WIDTH <= 20 and self.HEIGHT <= 20:
               self.BITMAP_FLAG = 0

        if self.OVERRIDE:
            self.odc.Clear()

            # Draw Balls again
            redraw_list = [True] * len(self.ball)
            pos_list = self.BALL_START
            self.draw_balls(redraw_list, pos_list)
            
    def on_update(self, x, y):
        if self.BITMAP_FLAG:
            redraw_list = [False] * len(self.ball)
            pos_list = [None] * len(self.ball)
            for i,ball in enumerate(self.ball):
                ball_point_distance = ((ball.x-x)**2 + (ball.y-y)**2)**(1/2.)
                if ball_point_distance < self.BALL_RADIUS and ball.x < self.BALL_RIGHTMARGIN:
                    if ball.x == self.BALL_LEFTMARGIN:
                        self.master_actuation_pub.publish(True)
                        self.master_force_pub.publish(True)
                        self.forcechannel_pub.publish(self.force_channel[i])
                        self.init_pause += 1

                    if self.init_pause >= self.INIT_WAIT:
                        self.wait_count[i] = 0
                        redraw_list[i] = True
                        pos_list[i] = [ball.x + self.BALL_MOVEX, ball.y]
                        break
                elif ball.x > self.BALL_START[i][0]:
                    if self.wait_count[i] < self.WAIT:
                        self.wait_count[i] += 1
                        break
                    else:
                        self.wait_count[i] = 0
                        redraw_list = [True] * len(self.ball)
                        pos_list = self.BALL_START
                        self.master_actuation_pub.publish(False)
                        self.master_force_pub.publish(False)
                        self.init_pause = 0

            if any(redraw_list):
                self.draw_balls(redraw_list, pos_list)

    def draw_balls(self, redraw_list, pos_list):
        dc = wx.ClientDC(self)
        self.odc.Clear()

        for i,redraw in enumerate(redraw_list):
            if redraw:
                self.ball[i].move_ball(dc, pos_list[i])

    def import_bitmap(self, dc, path):
        img  = path + 'start_'+str(self.WIDTH)+'_'+str(self.HEIGHT)+'.png'
        try:
            del self.bmp
            del self.bitmap
        except:
            pass

        if not os.path.exists(img):
            self.generate_png(dc, path)

        self.bmp = wx.Image(img, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        self.BITMAP_FLAG = True

    def generate_png(self, dc, path):
        if self.WIDTH > 20 and self.HEIGHT > 20:
            """set up the device for painting"""
            picture = wx.Bitmap(path + 'haptics_symp.png')
            width,height = picture.GetSize()
            dc.BeginDrawing()

            dc.DrawBitmap(picture,(self.WIDTH-width)/2,(self.HEIGHT-height)/2,True)
            dc.SetFont(wx.Font(self.TEXTBOX_FONTSIZE, wx.ROMAN, wx.FONTSTYLE_NORMAL, wx.NORMAL))
            """First Rectangle"""
            rectangle_y = self.FIRST_RECTANGLE_Y
            dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
            dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
            # set x, y, w, h for rectangle
            dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)

            """Second Rectangle"""
            rectangle_y = self.FIRST_RECTANGLE_Y + self.RECTANGLE_SEPERATION + self.RECTANGLE_SIZE
            dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
            dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
            # set x, y, w, h for rectangle
            dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)

            bmp = wx.EmptyBitmap(self.WIDTH, self.HEIGHT)
            memDC = wx.MemoryDC()
            memDC.SelectObject(bmp)
            memDC.Blit(0 ,0,self.WIDTH, self.HEIGHT, dc, 0,0)
            memDC.SelectObject(wx.NullBitmap)
            img = bmp.ConvertToImage()

            img.SaveFile(path + 'start_'+str(self.WIDTH)+'_'+str(self.HEIGHT)+'.png', wx.BITMAP_TYPE_PNG)

    def clearPanel(self):
        #clears panel when frame combo boxes are rolled up.
        self.odc.Clear()
        redraw_list = [True] * len(self.ball)
        pos_list = self.BALL_START
        self.draw_balls(redraw_list, pos_list)

if __name__ == '__main__':
    app = wx.App(False)
    frame = DemoFrame(velocity=5, refresh=10, length=19)
    frame.Show(True)
    app.MainLoop()


