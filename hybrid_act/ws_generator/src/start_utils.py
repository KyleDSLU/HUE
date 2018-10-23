#! /usr/bin/env python
import os
import time
import numpy as np

import wx
import main

class Ball(object):
    def __init__(self, l_xy, radius, x_lim, color="RED"):
        self.x = l_xy[0]
        self.y = l_xy[1]
        self.radius = radius
        self.color = color
        self.update_limit(x_lim)

    def move_forward(self, dc, velocity):
        if self.x < self.x_lim:
            self.x += velocity;
        self.draw(dc)

    def draw(self, dc):
        dc.SetPen(wx.Pen(self.color,style=wx.TRANSPARENT))
        dc.SetBrush(wx.Brush(self.color,wx.SOLID))
        dc.DrawCircle(self.x+5, self.y+5, self.radius)

    def update_limit(self, limit):
        self.x_lim = limit

    def move_ball(self, dc, l_xy):
        self.x = l_xy[0]
        self.y = l_xy[1]
        self.draw(dc)

    def hold_ball(self, dc):
        self.draw(dc)

class GuiPanel(wx.Panel):
    def __init__(self, parent, velocity, refresh, length):
        wx.Panel.__init__(self, parent)
        self.parent
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
        self.TEXTBOX_WIDTH = self.WIDTH
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
        self._layout()
        self.update_drawing()

    def update_drawing(self):
        self.Refresh(True)

    #go back
    def back_button(self,event):
        f = main.frameMain(None)
        self.Close()
        f.Show()

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

    def generate_ws(self,parent):
        intensity = np.zeros([4,self.HAPTIC_WIDTH])
        """Determine y workspace bounds"""
        y_ws = np.zeros([2,2])
        rectangle_y = self.FIRST_RECTANGLE_Y
        y_ws[0] = [rectangle_y,rectangle_y+self.RECTANGLE_SIZE]
        rectangle_y = rectangle_y + self.RECTANGLE_SIZE + self.RECTANGLE_SEPERATION
        y_ws[1] = [rectangle_y,rectangle_y + self.RECTANGLE_SIZE]

        HAPTIC_WIDTH = float(self.HAPTIC_WIDTH)
        HORIZ_PIXELS = np.arange(HAPTIC_WIDTH)

        # output has form of channel: actuation, amplitude, texture, frequency
        for i,value in enumerate(parent.rand_output.values()):
            amp = int(value[1]*100)
            if value[2] == "Sinusoid":
                if value[0] == "Hybrid":
                    sinusoid = amp*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*value[3]*2*np.pi)
                    ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]
                    intensity[2*i][ind[0]] = sinusoid[ind[0]]
                    intensity[2*i+1][ind[1]] = -sinusoid[ind[1]]
                elif value[0] == "UFM":
                    intensity[2*i] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*value[3]*2*np.pi) + amp/2
                elif value[0] == "EV":
                    intensity[2*i+1] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*value[3]*2*np.pi) + amp/2

            elif value[2]  == "Square":
                sinusoid = np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*value[3]*2*np.pi)
                ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]

                if value[0] == "Hybrid":
                    intensity[2*i][ind[0]] = amp
                    intensity[2*i+1][ind[1]] = amp
                elif value[0] == "UFM":
                    intensity[2*i][ind[0]] = amp
                elif value[0] == "EV":
                    intensity[2*i+1][ind[1]] = amp

        return intensity.astype(int).tolist(), y_ws.astype(int).tolist(), time.time()

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


