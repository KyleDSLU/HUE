#! /usr/bin/env python
import os
import time
import numpy as np

from ws_generator.msg import WSArray

import rospkg
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

class OptionList:
    def __init__(self, id, shape):
        """Constructor"""
        self.id = id
        self.shape = shape

class DemoPanel(wx.Panel):
    def __init__(self, parent, velocity, refresh, length):
        wx.Panel.__init__(self, parent)
        self.parent = parent
        self.ball = [[],[],[]]
        self.rospack = rospkg.RosPack()
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
        self.TEXTBOX_WIDTH = 0.20*self.WIDTH
        self.TEXTBOX_X = 0.04*self.WIDTH
        self.TEXTBOX_XOFFSET = 0.80*self.WIDTH
        self.TEXTBOX_Y = self.RECTANGLE_SIZE*0.35
        self.RECTANGLE_COLOR = "GREY"
        self.TEXTBOX_FONTSIZE = max(1,int(42*((self.WIDTH*self.HEIGHT)/(1794816.))))
        self.TEXTBOX_COLOR = "WHITE"

        self.HAPTIC_WIDTH = self.WIDTH - self.TEXTBOX_WIDTH

        self.BALL_RADIUS = int(self.RECTANGLE_SIZE/2*1.1)
        self.BALL_YOFFSET = -0.0125*self.HEIGHT
        self.BALL_LEFTMARGIN = 0.95*self.BALL_RADIUS+self.TEXTBOX_WIDTH
        self.BALL_RIGHTMARGIN = self.WIDTH-1.2*self.BALL_RADIUS
        first_ball_start = int(self.FIRST_RECTANGLE_Y+self.BALL_RADIUS+self.BALL_YOFFSET)
        second_ball_start = int(first_ball_start+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        #second_ball_start = int(self.FIRST_RECTANGLE_Y+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION+self.BALL_RADIUS+self.BALL_YOFFSET)
        third_ball_start = int(second_ball_start+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        self.BALL_START = [[self.BALL_LEFTMARGIN,first_ball_start],[self.BALL_LEFTMARGIN,second_ball_start],[self.BALL_LEFTMARGIN,third_ball_start]]

        for i in range(len(self.ball)):
            self.ball[i] = Ball(self.BALL_START[i],self.BALL_RADIUS,self.BALL_RIGHTMARGIN)

        self.BALL_MOVEX = int(self.BALL_VELOCITY*self.REFRESH*self.WIDTH/(2450.*self.LENGTH))
        self._layout()
        self.update_drawing()

    def update_drawing(self):
        self.Refresh(True)

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

    def on_select(self, event):
        texture = self.textures.GetStringSelection()
        Amplitude_EV = float(self.Amplitude_EV.GetStringSelection())/100
        Amplitude_UFM = float(self.Amplitude_UFM.GetStringSelection())/100
        Amplitude_H = float(self.Amplitude_H.GetStringSelection())/100
        freq=(int(self.Frequency.GetValue()))

        self.generate_workspace(texture,Amplitude_EV,Amplitude_UFM,Amplitude_H,freq)

    #go back
    def back_button(self,event):
        f = main.frameMain(None)
        self.Close()
        f.Show()

    def widgetMaker(self, widget, objects):
        """"""
        for obj in objects:
            widget.Append(obj.shape, obj)
        widget.Bind(wx.EVT_COMBOBOX, self.checking)

    def checking(self, event):
        if not (self.textures.GetStringSelection()=='' or self.Amplitude_EV.GetStringSelection()=='' or self.Amplitude_UFM.GetStringSelection()=='' or self.Amplitude_H.GetStringSelection()=='' or self.Frequency.GetValue()==''):
            self.SubmitBtn.Enable()

    def _background(self, evt, dc):
        """set up the device for painting"""
        # brush = wx.Brush(self.background_color)
        # dc.SetBackground(brush)
        # dc.Clear()

        #path = os.path.join('~/catkin_ws/src/hue/hybrid_act/ws_generator/ref', 'haptics_symp.png')
        path = self.rospack.get_path('ws_generator')
        path = os.path.join(path, 'ref/haptics_symp.png')
        picture = wx.Bitmap(path)
        width,height = picture.GetSize()

        dc.DrawBitmap(picture,(self.WIDTH-width)/2,(self.HEIGHT-height)/2,True)
        dc.SetFont(wx.Font(self.TEXTBOX_FONTSIZE, wx.ROMAN, wx.FONTSTYLE_NORMAL, wx.NORMAL))

        """EV Rectangle"""
        rectangle_y = self.FIRST_RECTANGLE_Y
        dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
        dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
        # set x, y, w, h for rectangle
        dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)
        dc.SetPen(wx.Pen(self.TEXTBOX_COLOR))
        dc.SetBrush(wx.Brush(self.TEXTBOX_COLOR))
        dc.DrawRectangle(0, rectangle_y, self.TEXTBOX_WIDTH, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X, rectangle_y+self.TEXTBOX_Y)
        dc.DrawLabel("EV", textbox, alignment=1)

        """UFM Rectangle"""
        rectangle_y = self.FIRST_RECTANGLE_Y+(self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
        dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
        # set x, y, w, h for rectangle
        dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)
        dc.SetPen(wx.Pen(self.TEXTBOX_COLOR))
        dc.SetBrush(wx.Brush(self.TEXTBOX_COLOR))
        dc.DrawRectangle(0, rectangle_y, self.TEXTBOX_WIDTH, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X, rectangle_y+self.TEXTBOX_Y)
        dc.DrawLabel("UFM", textbox, alignment=1)

        """Hybrid Rectangle"""
        rectangle_y = self.FIRST_RECTANGLE_Y+(self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)*2
        dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
        dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
        # set x, y, w, h for rectangle
        dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)
        dc.SetPen(wx.Pen(self.TEXTBOX_COLOR))
        dc.SetBrush(wx.Brush(self.TEXTBOX_COLOR))
        dc.DrawRectangle(0, rectangle_y, self.TEXTBOX_WIDTH, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X, rectangle_y+self.TEXTBOX_Y)
        dc.DrawLabel("Hybrid", textbox, alignment=1)

    def _layout(self):#set up buttons and texts
        Frequency_label=wx.StaticText(self,wx.ID_ANY,pos=(0.7*self.WIDTH,0.02*self.HEIGHT),label='Frequency (Hz)')
        self.Frequency= wx.TextCtrl(self, wx.ID_ANY, pos=(0.7*self.WIDTH,0.05*self.HEIGHT))
        self.Frequency.Bind(wx.EVT_TEXT, self.checking)

        self.BackBtn = wx.Button(self,wx.ID_ANY,label='BACK',pos=(0,0))
        self.BackBtn.Bind(wx.EVT_BUTTON,self.back_button)

        self.SubmitBtn = wx.Button(self,wx.ID_ANY,label='Sumbit',pos=(0.8*self.WIDTH,0.05*self.HEIGHT))
        self.SubmitBtn.Bind(wx.EVT_BUTTON,self.on_select)

        textures = [OptionList(0, "Bump"),
                    OptionList(1, "Sinusoidal"),
                    OptionList(2, "Triangular"),
                    OptionList(3, "Square")]

        sampleList = []

        texture_label=wx.StaticText(self,wx.ID_ANY,pos=(0.3*self.WIDTH,0.02*self.HEIGHT),label='Texture')
        self.textures = wx.ComboBox(self,
                              size=wx.DefaultSize,
                              choices=sampleList,
                              pos=(0.3*self.WIDTH,0.05*self.HEIGHT))
        self.widgetMaker(self.textures, textures)

        amplitudes=[OptionList(0,'10'), \
                   OptionList(1,'20'), \
                   OptionList(2,'30'), \
                   OptionList(3,'40'), \
                   OptionList(4,'50'), \
                   OptionList(5,'60'), \
                   OptionList(6,'70'), \
                   OptionList(7,'80'), \
                   OptionList(8,'90'), \
                   OptionList(9,'100')]

        EV_label=wx.StaticText(self,wx.ID_ANY,pos=(0.4*self.WIDTH,0.02*self.HEIGHT),label='EV Amplitude')
        self.Amplitude_EV = wx.ComboBox(self,
                              size=wx.DefaultSize,
                              choices=sampleList,
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT))
        self.widgetMaker(self.Amplitude_EV, amplitudes)

        UFM_label=wx.StaticText(self,wx.ID_ANY,pos=(0.5*self.WIDTH,0.02*self.HEIGHT),label='UFM Amplitude')
        self.Amplitude_UFM = wx.ComboBox(self,
                              size=wx.DefaultSize,
                              choices=sampleList,
                              pos=(0.5*self.WIDTH,0.05*self.HEIGHT))
        self.widgetMaker(self.Amplitude_UFM, amplitudes)

        H_label=wx.StaticText(self,wx.ID_ANY,pos=(0.6*self.WIDTH,0.02*self.HEIGHT),label='Hybrid Amplitude')
        self.Amplitude_H = wx.ComboBox(self,
                              size=wx.DefaultSize,
                              choices=sampleList,
                              pos=(0.6*self.WIDTH,0.05*self.HEIGHT))
        self.widgetMaker(self.Amplitude_H, amplitudes)

        self.SubmitBtn.Disable()

    def generate_workspace(self,texture,Amplitude_EV,Amplitude_UFM,Amplitude_H,frequency):
        """Determine y workspace bounds"""
        y_ws_ufm = []
        y_ws_ev = []

        rectangle_y = self.FIRST_RECTANGLE_Y

        y_ws_ev.append(rectangle_y)
        y_ws_ev.append(rectangle_y+self.RECTANGLE_SIZE)

        rectangle_y = rectangle_y + self.RECTANGLE_SIZE + self.RECTANGLE_SEPERATION
        y_ws_ufm.append(rectangle_y)
        y_ws_ufm.append(rectangle_y + self.RECTANGLE_SIZE)

        rectangle_y = rectangle_y + self.RECTANGLE_SIZE + self.RECTANGLE_SEPERATION
        y_ws_ev.append(rectangle_y)
        y_ws_ev.append(rectangle_y + self.RECTANGLE_SIZE)
        y_ws_ufm.append(rectangle_y)
        y_ws_ufm.append(rectangle_y + self.RECTANGLE_SIZE)

        haptic_width = float(self.HAPTIC_WIDTH)
        horiz_pixels = np.arange(haptic_width)

        """Determine x intensities correlating with texture"""
        if texture == "Bump":
            x_center = (haptic_width)/2
            x_biasedcenter = x_center+self.TEXTBOX_WIDTH

            x_haptic_switch = [x_biasedcenter-225,x_biasedcenter+225]
            x_ufm_dropoff = [x_biasedcenter-400,x_biasedcenter+400]
            x_ev_max = [x_biasedcenter-75,x_biasedcenter+75]

            c1 = (x_haptic_switch[0]-x_biasedcenter)**2
            c2 = (x_ufm_dropoff[0]-x_biasedcenter)**2
            kufm = -c1/(c2-c1)
            aufm = (1.0-kufm)/c2

            c3 = (x_ev_max[0]-x_biasedcenter)**2
            kev = -c1/(c3-c1)
            aev = (1.0-kev)/c3

            """Set haptic intensity = 0 over textbox"""
            ufm_intensity = np.zeros(self.WIDTH)
            ev_intensity = np.zeros(self.WIDTH)

            for index in range(int(self.TEXTBOX_WIDTH),int(self.WIDTH)):
                #print(x_ufm_dropoff, index)
                if (index <= x_ufm_dropoff[0] or index >= x_ufm_dropoff[1]):
                    ufm_intensity[index] = 1.0
                    ev_intensity[index] = 0.0

                else:
                    ufm_int = aufm*(index-x_biasedcenter)**2+kufm
                    ufm_intensity[index] = max(0,min(1,ufm_int))

                    ev_int = aev*(index-x_biasedcenter)**2+kev
                    ev_intensity[index] = max(0,min(1,ev_int))

            hybridev_intensity = ev_intensity
            hybridufm_intensity = ufm_intensity

        elif texture == "Sinusoidal":
            hybridufm_intensity = np.zeros(len(haptic_width))
            hybridev_intensity = np.zeros(len(haptic_width))
            sinusoid = np.sin(horiz_pixels/haptic_width*frequency*2*np.pi)
            ind = [np.where(sinusoid>0)[0],np.where(sinusoid<=0)[0]]
            hybridufm_intensity[ind[0]] = sinusoid[ind[0]]
            hybridev_intensity[ind[1]] = -sinusoid[ind[1]]

            positive_sin = sinusoid/2.+0.5
            ufm_intensity = positive_sin
            ev_intensity = positive_sin

            """Set haptic intensity = 0 over textbox"""
            hybridufm_intensity = np.append(zeros(self.TEXTBOX_WIDTH),hybridufm_intensity)
            hybridev_intensity = np.append(zeros(self.TEXTBOX_WIDTH),hybridev_intensity)
            ufm_intensity = np.append(zeros(self.TEXTBOX_WIDTH),ufm_intensity)
            ev_intensity = np.append(zeros(self.TEXTBOX_WIDTH),ev_intensity)

        elif texture == "Triangular":
            hybridufm_intensity = np.zeros(len(haptic_width))
            hybridev_intensity = np.zeros(len(haptic_width))
            triangle = np.zeros(len(haptic_width))
            slope = 2.0*frequency/haptic_width
            triangle[0] = -1.0
            for i in range(1,len(triangle)):
                if triangle[i-1] >= 1.0:
                    triangle[i] = -1.0
                else:
                    triangle[i] = min(1.0,triangle[i]+slope)
            ind = [np.where(triangle>0)[0], np.where(triangle<=0)[0]]
            hybridufm_intensity[ind[1]] = triangle[ind[1]]
            hybridev_intensity[ind[0]] = triangle[ind[0]]

            positive_triangle = triangle/2.+0.5
            ufm_intensity = positive_triangle
            ev_intensity = positive_triange

            """Set haptic intensity = 0 over textbox"""
            hybridufm_intensity = np.append(zeros(self.TEXTBOX_WIDTH),hybridufm_intensity)
            hybridev_intensity = np.append(zeros(self.TEXTBOX_WIDTH),hybridev_intensity)
            ufm_intensity = np.append(zeros(self.TEXTBOX_WIDTH),ufm_intensity)
            ev_intensity = np.append(zeros(self.TEXTBOX_WIDTH),ev_intensity)

        elif texture == "Square":
            hybridufm_intensity = np.zeros(len(haptic_width))
            hybridev_intensity = np.zeros(len(haptic_width))
            sinusoid = np.sin(2*np.pi*frequency*horiz_pixels/haptic_width)
            ind = [np.where(sinusoid>0)[0],np.where<=0)[0]]
            square = np.zeros(len(sinusoid))
            square[ind[0]] = 1
            square[ind[1]] = -1
            hybridufm_intensity[ind[1]] = -square[ind[1]]
            hybridev_intensity[ind[0]] = square[ind[0]]
            ufm_intensity = (1+square)/2
            ev_intensity = (1+square)/2

            """Set haptic intensity = 0 over textbox"""
            hybridufm_intensity = np.append(zeros(self.textbox_width),hybridufm_intensity)
            hybridev_intensity = np.append(zeros(self.textbox_width),hybridev_intensity)
            ufm_intensity = np.append(zeros(self.textbox_width),ufm_intensity)
            ev_intensity = np.append(zeros(self.textbox_width),ev_intensity)


        ufm_intensity = np.append(ufm_amplitde*ufm_intensity,hybrid_amplitude*hybridufm_intensity)
        ev_intensity = np.append(ev_amplitde*ev_intensity,hybrid_amplitude*hybridev_intensity)
        intensity = np.append(ufm_intensity,ev_intensity)
        y_ws = np.append(y_ws_ufm,y_ws_ev)

        ev_msg = WSArray()
        ev_msg.header.stamp = rospy.Time(0.0)
        ev_msg.y_step = 2
        ev_msg.y_ws = y_ws_ev
        ev_msg.int_step = 2
        ev_msg.intensity = ev_intensity.astype(int).tolist()

        ufm_msg = WSArray()
        ufm_msg.header.stamp = rospy.Time(0.0)
        ufm_msg.y_step = 2
        ufm_msg.y_ws = y_ws_ufm
        ufm_msg.int_step = 2
        ufm_msg.intensity = ufm_intensity.astype(int).tolist()

        self.parent.ws_ufm_pub.publish(ufm_msg)
        self.parent.ws_ev_pub.publish(ev_msg)

class DemoFrame(wx.Frame):
    def __init__(self, velocity, refresh, length, *args, **kw):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = wx.EmptyString, size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)

        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)

        self.panel = DemoPanel(self, velocity, refresh, length)
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)

    def on_close(self, event):
        self.timer.Stop()
        self.Destroy()

    def on_timer(self, event):
        self.panel.update_drawing()

if __name__ == '__main__':
    app = wx.App(False)
    frame = DemoFrame(10, 20, 15)
    frame.Show(True)
    app.MainLoop()


