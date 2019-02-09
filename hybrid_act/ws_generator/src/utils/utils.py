#! /usr/bin/env/python3
import numpy as np
import wx
import rospy
from std_msgs.msg import Float32, String

np.set_printoptions(suppress=True)

class QuestionPanel(wx.Panel):
    def __init__(self, parent, fontsize):
        wx.Panel.__init__(self, parent)
        font = wx.Font(fontsize, wx.ROMAN, wx.NORMAL, wx.NORMAL)

        width = fontsize*75
        height = fontsize*6
        self.innerPanel = wx.Panel(self, -1, style=wx.ALIGN_CENTER)
        hbox = wx.BoxSizer(wx.HORIZONTAL) 
        vbox = wx.BoxSizer(wx.VERTICAL)
        innerBox = wx.BoxSizer(wx.VERTICAL)

        # I want this line visible in the CENTRE of the inner panel
        self.txt = wx.StaticText(self.innerPanel, id=-1, label="XXXXXXXXXXXXXXXXXX", style=wx.ALIGN_LEFT, name="")
        self.txt.SetFont(font)
        innerBox.AddSpacer((width,height/3))
        innerBox.Add(self.txt, 0, wx.LEFT)
        innerBox.AddSpacer((width,height/3))
        self.innerPanel.SetSizer(innerBox)

        hbox.Add(self.innerPanel, 0, wx.ALL|wx.ALIGN_CENTER)
        vbox.Add(hbox, 1, wx.ALL|wx.ALIGN_LEFT, 5)

        self.SetSizer(vbox)
        vbox.Fit(self)

        self.question_sub = rospy.Subscriber('/user_study/question', String, self.question_callback, queue_size = 1)

    def question_callback(self, question):
        question = question.data
        wx.CallAfter(self.txt.SetLabelText, question)

class NormForceLabel(wx.Panel):
    def __init__(self, parent, fontsize):
        wx.Panel.__init__(self, parent)
        font = wx.Font(fontsize, wx.ROMAN, wx.NORMAL, wx.NORMAL)

        width = fontsize*25
        height = fontsize*6
        self.innerPanel = wx.Panel(self, -1, style=wx.ALIGN_CENTER)
        hbox = wx.BoxSizer(wx.HORIZONTAL) 
        vbox = wx.BoxSizer(wx.VERTICAL)
        innerBox = wx.BoxSizer(wx.VERTICAL)

        # I want this line visible in the CENTRE of the inner panel
        self.txt = wx.StaticText(self.innerPanel, id=-1, label="Pressing Force %", style=wx.ALIGN_CENTER, name="")
        self.txt.SetFont(font)
        innerBox.AddSpacer((width,height/3))
        innerBox.Add(self.txt, 0, wx.CENTER)
        innerBox.AddSpacer((width,height/3))
        self.innerPanel.SetSizer(innerBox)

        hbox.Add(self.innerPanel, 0, wx.ALL|wx.ALIGN_CENTER)
        vbox.Add(hbox, 1, wx.ALL|wx.ALIGN_CENTER, 5)

        self.SetSizer(vbox)
        vbox.Fit(self)

class NormForcePanel(wx.Panel):
    def __init__(self, parent, fontsize, norm_force_desired, norm_force_threshold):
        wx.Panel.__init__(self, parent)
        font = wx.Font(fontsize, wx.ROMAN, wx.NORMAL, wx.NORMAL)

        self.norm_force_desired = norm_force_desired
        self.norm_force_threshold = norm_force_threshold

        width = fontsize*14
        height = fontsize*6
        self.innerPanel = wx.Panel(self, -1, style=wx.ALIGN_CENTER)
        self.innerPanel.SetBackgroundColour('WHITE')
        hbox = wx.BoxSizer(wx.HORIZONTAL) 
        vbox = wx.BoxSizer(wx.VERTICAL)
        innerBox = wx.BoxSizer(wx.VERTICAL)

        # I want this line visible in the CENTRE of the inner panel
        self.txt = wx.StaticText(self.innerPanel, id=-1, label="100.0", style=wx.ALIGN_CENTER, name="")
        self.txt.SetFont(font)
        innerBox.AddSpacer((width,height/3))
        innerBox.Add(self.txt, 0, wx.CENTER)
        innerBox.AddSpacer((width,height/3))
        self.innerPanel.SetSizer(innerBox)

        hbox.Add(self.innerPanel, 0, wx.ALL|wx.ALIGN_CENTER)
        vbox.Add(hbox, 1, wx.ALL|wx.ALIGN_CENTER, 5)

        self.SetSizer(vbox)
        vbox.Fit(self)

        self.norm_force_sub = rospy.Subscriber('/force_recording/normal_force', Float32, self.norm_force_callback, queue_size = 1)

    def norm_force_callback(self, norm_force):
        norm_force = norm_force.data
        norm_force_percent = float(100*norm_force/self.norm_force_desired)
        wx.CallAfter(self.txt.SetLabel, "{0:.1f}".format(norm_force_percent))
        if ((100-self.norm_force_threshold) <= norm_force_percent <= (100+self.norm_force_threshold)):
            color = "GREEN"
        else:
            color = "RED"
        wx.CallAfter(self.innerPanel.SetBackgroundColour, color)

class Ball(object):
    def __init__(self, l_xy, radius, x_lim, color="RED"):
        self.x = l_xy[0]
        self.y = l_xy[1]
        self.radius = radius
        self.color = color
        self.update_limit(x_lim)

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

def Generate_WS(obj, ws, ws_div):
    WS_DIV = ws_div
    WS_SIZE = int(np.ceil(obj.WIDTH/float(WS_DIV)))

    # Intitialize int arrays
    intensity = np.array([[.5]*obj.WIDTH]*len(ws))

    """Determine y workspace bounds"""
    y_ws = np.empty([len(ws),2])
    rectangle_y = obj.FIRST_RECTANGLE_Y
    for i in range(len(ws)):
        y_ws[i] = [rectangle_y,rectangle_y+obj.RECTANGLE_SIZE]
        if len(ws) > 1:
            rectangle_y = rectangle_y + obj.RECTANGLE_SIZE + obj.RECTANGLE_SEPERATION

    HAPTIC_WIDTH = float(obj.HAPTIC_WIDTH)
    HORIZ_PIXELS = np.arange(HAPTIC_WIDTH)

    # ws has form of channel: actuation, amplitude, texture, frequency
    for i,value in enumerate(ws.values()):
        actuation = value[0]
        amp = value[1]
        shape = value[2]
        frequency = value[3]

        if shape == "Sinusoid":
            intensity[i][int(obj.WIDTH-obj.HAPTIC_WIDTH):] = amp/2.*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi) + .5

        elif shape  == "Square":
            sinusoid = np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi)
            ind = [np.where(sinusoid>=0)[0], np.where(sinusoid<0)[0]]
            intensity[i][int(ind[0]+obj.TEXTBOX_WIDTH)] = amp/2. + 0.5
            intensity[i][int(ind[1]+obj.TEXTBOX_WIDTH)] = 0.5 - amp/2.

        elif shape == "Triangular":
            triangle = np.zeros(obj.HAPTIC_WIDTH)
            slope = frequency/float(obj.HAPTIC_WIDTH)
            triangle[0] = 0.5 - amp/2.
            for j in range(1,len(triangle)):
                if triangle[j-1] >= amp/2. + 0.5:
                    triangle[j] = 0.5 - amp/2.
                else:
                    triangle[j] = min(amp/2.+0.5, triangle[j-1]+slope)

            intensity[i][int(obj.WIDTH-obj.HAPTIC_WIDTH):] = triangle

        elif shape == "Bump":
            center = (HAPTIC_WIDTH)/2
            biasedcenter = center+obj.TEXTBOX_WIDTH

            haptic_switch = [biasedcenter-int(0.375*HAPTIC_WIDTH),biasedcenter+int(0.375*HAPTIC_WIDTH)]
            haptic_max = [biasedcenter-int(0.1*HAPTIC_WIDTH),biasedcenter+int(0.1*HAPTIC_WIDTH)]

            # Solve parabolic formula for a, k when y(x=haptic_switch) = 0
            # and y(x=haptic_max) = 1
            c1 = (haptic_switch[0]-biasedcenter)**2
            c2 = (haptic_max[0]-biasedcenter)**2
            a = (0.5 + amp/2.)/(c1-c2)
            k = a*c1 - (0.5 - amp/2.)

            for index in range(int(obj.WIDTH-obj.HAPTIC_WIDTH),int(obj.WIDTH)):
                if (index <= haptic_switch[0] or index >= haptic_switch[1]):
                    intensity[i][index] = (0.5 - amp/2.)
                else:
                    value = -a*(index-biasedcenter)**2+k
                    intensity[i][index] = max((0.5-amp/2.),min((0.5+amp/2.),value))

    intensity_out = np.zeros([len(intensity),WS_SIZE])
    for j in range(len(intensity)):
        intensity_out[j][:] = intensity[j][::WS_DIV]*100

    return intensity_out.astype(int), y_ws.astype(int)

