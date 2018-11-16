#! /usr/bin/env python
import os
import time
import numpy as np

import wx

from utils import Ball, WS_Generator

class GuiPanel(wx.Panel):
    def __init__(self, parent, velocity, refresh, length):
        wx.Panel.__init__(self, parent)
        self.parent = parent
        self.ball = [[],[]]
        self.last_pos = self.ScreenToClient(wx.GetMousePosition())
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.SetBackgroundColour("BLACK")

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.REFRESH = refresh
        self.LENGTH = length
        self.BALL_VELOCITY = velocity

        self.WAIT = 10
        self.wait_count = [0]*len(self.ball)

        self.update_drawing()
        self.on_size(0)

        wx.CallLater(200, self.SetFocus)

    def on_size(self, event):
        self.WIDTH, self.HEIGHT = self.GetClientSize()
        self.WIDTH_HALF = self.WIDTH/2.0
        self.FIRST_RECTANGLE_Y = 0.1*self.HEIGHT
        self.BOTTOM_SPACE = 0.035*self.HEIGHT

        self.RECTANGLE_SIZE = 0.175*self.HEIGHT
        self.RECTANGLE_SEPERATION = int((self.HEIGHT-self.FIRST_RECTANGLE_Y-self.BOTTOM_SPACE-3*self.RECTANGLE_SIZE)/3.0)

        #set up textbox and lines
        self.TEXTBOX_WIDTH = 0
        self.TEXTBOX_X = 0.04*self.WIDTH
        self.TEXTBOX_XOFFSET = 0.80*self.WIDTH
        self.TEXTBOX_Y = self.RECTANGLE_SIZE*0.35
        self.RECTANGLE_COLOR = "WHITE"
        self.TEXTBOX_FONTSIZE = max(1,int(42*((self.WIDTH*self.HEIGHT)/(1794816.))))

        self.HAPTIC_WIDTH = self.WIDTH

        self.BALL_RADIUS = int(self.RECTANGLE_SIZE/2*1.1)
        self.BALL_YOFFSET = -0.0125*self.HEIGHT
        self.BALL_MARGIN = self.BALL_RADIUS*1.2
        self.BALL_START = [[self.BALL_MARGIN,int(self.FIRST_RECTANGLE_Y+self.BALL_RADIUS+self.BALL_YOFFSET)],[self.BALL_MARGIN,int(self.FIRST_RECTANGLE_Y+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION+self.BALL_RADIUS+self.BALL_YOFFSET)]]
        # self._buffer = wx.Bitmap(self.WIDTH, self.HEIGHT)
        for i in range(len(self.ball)):
            self.ball[i] = Ball(self.BALL_START[i],self.BALL_RADIUS,self.WIDTH-self.BALL_MARGIN)

        self.BALL_MOVEX = int(self.BALL_VELOCITY*self.REFRESH*self.WIDTH/(2450.*self.LENGTH))
        self.ws_gen = WS_Generator(self)

    def update_drawing(self):
        self.Refresh(True)

    #go back
    def back_button(self,event):
        self.Close()
        self.parent.close()

    def on_paint(self, event):
        x, y = self.ScreenToClient(wx.GetMousePosition())
        dc = wx.AutoBufferedPaintDC(self)
        dc.Clear()
        self._background(event,dc)

        for i,ball in enumerate(self.ball):
            if ball.x - self.BALL_RADIUS <= x <= ball.x + self.BALL_RADIUS:
                if ball.y - self.BALL_RADIUS <= y <= ball.y + self.BALL_RADIUS:
                    self.wait_count[i] = 0
                    ball.move_forward(dc,self.BALL_MOVEX)
                    parent.publish_master_status(True,True)
                    break
            elif ball.x != self.BALL_START[i][0]:
                if self.wait_count[i] < self.WAIT:
                    self.wait_count[i] += 1
                    ball.hold_ball(dc)
                    break
                else:
                    self.wait_count[i] = 0
                    ball.move_ball(dc,self.BALL_START[i])
            else:
                ball.move_ball(dc,self.BALL_START[i])
                parent.publish_force_status(False,False)

    def generate_ws(self,ws):
        return self.ws_gen.ws_generator(ws), time.time

    def _background(self, evt, dc):
        """set up the device for painting"""
        dc.SetFont(wx.Font(self.TEXTBOX_FONTSIZE, wx.ROMAN, wx.FONTSTYLE_NORMAL, wx.NORMAL))

        """Rectangle 1"""
        RECTANGLE_Y = self.FIRST_RECTANGLE_Y
        dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
        dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
        dc.DrawRectangle(0,RECTANGLE_Y, self.WIDTH, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X+self.TEXTBOX_XOFFSET, RECTANGLE_Y+self.TEXTBOX_Y)
        dc.DrawLabel("(1)", textbox, alignment=1)

        """Rectangle 2"""
        RECTANGLE_Y = self.FIRST_RECTANGLE_Y+(self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        dc.DrawRectangle(0,RECTANGLE_Y, self.WIDTH, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X+self.TEXTBOX_XOFFSET, RECTANGLE_Y+self.TEXTBOX_Y)
        dc.DrawLabel("(2)", textbox, alignment=0)

    def _layout(self):#set up buttons and texts
        try:
            if self.tc[0] == 1:
                gui_question = "Which Texture Felt Stronger?"
        except:
            gui_question = ''

        back_button = wx.Button(self,wx.ID_ANY,'BACK')
        label = wx.StaticText(self,wx.ID_ANY,label=gui_question,pos=(0,.7*self.HEIGHT))
        label.SetFont(wx.Font(self.TEXTBOX_FONTSIZE,wx.ROMAN,wx.NORMAL,wx.BOLD))
        button_1 = wx.Button(self,wx.ID_ANY, '1',pos=(0,self.HEIGHT*.8),size=(self.WIDTH_HALF,.2*self.HEIGHT))
        button_2 = wx.Button(self,wx.ID_ANY, '2',pos=(self.WIDTH_HALF,self.HEIGHT*.8),size=(self.WIDTH_HALF,.2*self.HEIGHT))

        #set up font
        button_1.SetFont(wx.Font(self.TEXTBOX_FONTSIZE,wx.ROMAN,wx.NORMAL,wx.BOLD))
        button_2.SetFont(wx.Font(self.TEXTBOX_FONTSIZE,wx.ROMAN,wx.NORMAL,wx.BOLD))

        ##connect buttons to functions
        back_button.Bind(wx.EVT_BUTTON,self.back_button)
        #add arguments when connecting buttons to functions using lambda
        button_1.Bind(wx.EVT_BUTTON,lambda evt: self.option(evt,1))
        button_2.Bind(wx.EVT_BUTTON,lambda evt: self.option(evt,2))
        #self.on_paint(0)#draw again

class GuiFrame(wx.Frame):
    def __init__(self, velocity, refresh, length, *args, **kw):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = wx.EmptyString, size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)

        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)

        self.panel = GuiPanel(self, velocity, refresh, length)
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)

    def on_close(self, event):
        self.timer.Stop()
        self.Destroy()

    def on_timer(self, event):
        self.panel.update_drawing()

if __name__ == '__main__':
    app = wx.App(False)
    frame = GuiFrame(10,20,15)
    frame.Show(True)
    app.MainLoop()


