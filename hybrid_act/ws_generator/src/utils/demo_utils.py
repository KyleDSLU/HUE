#! /usr/bin/env python
import os
import time 
import numpy as np 
from ws_generator.msg import WSArray
import time
import rospkg
import rospy
import wx
import main

from utils import Ball, Generate_WS

class OptionList:
    def __init__(self, id, shape):
        """Constructor"""
        self.id = id
        self.shape = shape

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
        self.LENGTH = length
        self.BALL_VELOCITY = velocity
        self.screenDC = wx.ScreenDC()

        self.BITMAP_FLAG = False
        self.updateFlag = False
        self.WAIT = 10

        self.on_size(0)
        self.update_drawing()
        wx.CallLater(200, self.SetFocus)


    def on_size(self, event):
        self.WIDTH, self.HEIGHT = self.GetClientSize()
        self.WIDTH_HALF = self.WIDTH/2.0
        self.FIRST_RECTANGLE_Y = 0.4*self.HEIGHT
        self.BOTTOM_SPACE = 0.035*self.HEIGHT

        self.RECTANGLE_SIZE = .25*self.HEIGHT
        self.RECTANGLES = 1
        self.RECTANGLE_SEPERATION = int((self.HEIGHT-self.FIRST_RECTANGLE_Y-self.BOTTOM_SPACE-self.RECTANGLES*self.RECTANGLE_SIZE)/3.0)

        #set up textbox and lines
        self.TEXTBOX_WIDTH = 0.15*self.WIDTH
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
        self.BALL_RIGHTMARGIN = self.WIDTH-1.2*self.BALL_RADIUS
        self.ball = [[]]
        self.wait_count = [0]*len(self.ball)
        first_ball_start = int(self.FIRST_RECTANGLE_Y+self.BALL_RADIUS+self.BALL_YOFFSET)
        # second_ball_start = int(first_ball_start+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        self.BALL_START = [[self.BALL_LEFTMARGIN,first_ball_start]]#,[self.BALL_LEFTMARGIN,second_ball_start],[self.BALL_LEFTMARGIN,third_ball_start]]

        for i in range(len(self.ball)):
            self.ball[i] = Ball(self.BALL_START[i],self.BALL_RADIUS,self.BALL_RIGHTMARGIN)

        self.BALL_MOVEX = int(self.BALL_VELOCITY*self.REFRESH*self.WIDTH/(2450.*self.LENGTH))

    def update_drawing(self):
        self.on_update()

    def on_paint(self, event):
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

    def on_update(self):
        if self.BITMAP_FLAG:
            x, y = self.ScreenToClient(wx.GetMousePosition())
            redraw_list = [False] * len(self.ball)
            pos_list = [None] * len(self.ball)
            for i,ball in enumerate(self.ball):
                if ball.x - self.BALL_RADIUS <= x <= ball.x + self.BALL_RADIUS:
                    if ball.y - self.BALL_RADIUS <= y <= ball.y + self.BALL_RADIUS:
                        self.wait_count[i] = 0
                        redraw_list[i] = True
                        pos_list[i] = [ball.x + self.BALL_MOVEX, ball.y]
                        break
                elif ball.x != self.BALL_START[i][0]:
                    if self.wait_count[i] < self.WAIT:
                        self.wait_count[i] += 1
                        break
                    else:
                        self.wait_count[i] = 0
                        redraw_list = [True] * len(self.ball)
                        pos_list = self.BALL_START
            if any(redraw_list):
                self.draw_balls(redraw_list, pos_list)

    def draw_balls(self, redraw_list, pos_list):
        dc = wx.ClientDC(self)
        self.odc.Clear()

        for i,redraw in enumerate(redraw_list):
            if redraw:
                self.ball[i].move_ball(dc, pos_list[i])

    def import_bitmap(self, dc, path):
        img  = path + 'demo_'+str(self.WIDTH)+'_'+str(self.HEIGHT)+'.png'
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
            """Hybrid Rectangle"""
            rectangle_y = self.FIRST_RECTANGLE_Y
            dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
            dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
            # set x, y, w, h for rectangle
            dc.DrawRectangle(0, rectangle_y, self.WIDTH, self.RECTANGLE_SIZE)
            dc.SetPen(wx.Pen(self.TEXTBOX_COLOR))
            dc.SetBrush(wx.Brush(self.TEXTBOX_COLOR))
            dc.DrawRectangle(0, rectangle_y, self.TEXTBOX_WIDTH, self.RECTANGLE_SIZE)
            textbox = wx.Rect(self.TEXTBOX_X, rectangle_y+self.TEXTBOX_Y)
            dc.DrawLabel("Hybrid", textbox, alignment=1)

            bmp = wx.EmptyBitmap(self.WIDTH, self.HEIGHT)
            memDC = wx.MemoryDC()
            memDC.SelectObject(bmp)
            memDC.Blit(0 ,0,self.WIDTH, self.HEIGHT, dc, 0,0)
            memDC.SelectObject(wx.NullBitmap)
            img = bmp.ConvertToImage()

            img.SaveFile(path + 'demo_'+str(self.WIDTH)+'_'+str(self.HEIGHT)+'.png', wx.BITMAP_TYPE_PNG)


    def on_select(self, event):
        texture = self.textures.GetStringSelection()
        Amplitude_EV = float(self.Amplitude_EV.GetStringSelection())/100
        Amplitude_UFM = float(self.Amplitude_UFM.GetStringSelection())/100
        Amplitude_H = float(self.Amplitude_H.GetStringSelection())/100
        freq=(int(self.Frequency.GetValue()))

        self.generate_workspace(texture,Amplitude_EV,Amplitude_UFM,Amplitude_H,freq)


    def widgetMaker(self, widget, objects):
        """"""
        for obj in objects:
            widget.Append(obj.shape, obj)
        widget.Bind(wx.EVT_COMBOBOX, self.checking)

    def checking(self, event):
        if not (self.textures.GetStringSelection()=='' or self.Amplitude_EV.GetStringSelection()=='' or self.Amplitude_UFM.GetStringSelection()=='' or self.Amplitude_H.GetStringSelection()=='' or self.Frequency.GetValue()==''):
            self.SubmitBtn.Enable()

    def clearPanel(self):
        #clears panel when frame combo boxes are rolled up.
        self.odc.Clear()
        redraw_list = [True] * len(self.ball)
        pos_list = self.BALL_START
        self.draw_balls(redraw_list, pos_list)


class DemoFrame(wx.Frame):
    def __init__(self, velocity, refresh, length, *args, **kw):
        self._layout(velocity, refresh, length)
        self.time = wx.DateTime.Now()
	#self.Bind(wx.EVT_TIMER, self.)
	self.update_normal_force()

    def update_normal_force(self, *args):
	self.time = wx.DateTime.Now()
	self.frequency.ChangeValue(self.time.Format("%c", wx.DateTime.CST))
	wx.CallLater(1000, self.update_normal_force)	
    def update_panel(self,evt):
        try:
            self.panel.updateFlag = True  
        except AttributeError:
            pass 
            #catching error if panel attribute does not exist i

    def on_back_button(self,event):
        #checks if the panel needs updated after the frame dropdowns have covered it
        f = main.frameMain(None)
        self.Close()
        f.Show()

    def checkSelect(self, evt):
        #checks to see if all selections have been made before submit button is enabled
        if not ((self.frequency.GetValue() == '') or (self.texture.GetValue() == '') or (self.Amplitude_hybrid.GetValue() == '')):
            self.submit_button.Enable()            

    def widgetMaker(self, widget, objects):
        #helps with makeing combobox lists
        for obj in objects:
            widget.Append(obj.shape, obj)

    def on_close(self, event):
        self.timer.Stop()
        self.Destroy()

    def on_timer(self, event):
        self.panel.update_drawing()

    def _layout(self, velocity, refresh, length):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = "Haptic Demo", size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)
        WIDTH, HEIGHT = self.GetClientSize()
        FONTSCALING = 0.01
        ICONSCALING = 0.01
        COMBOSCALING = ICONSCALING*1.1
        HORIZSPACERSIZE = 4
        VERTSPACERMULT = HEIGHT*0.01

        LEFTTEXTSPACING = int(WIDTH*0.008)
        BUTTONTEXTSPACING = int(WIDTH*0.007)
        COMBOTEXTSPACING = int(WIDTH*0.0135)
        TEXTOFFSETS = [0,0,0,3,2]

        icon_size = WIDTH*ICONSCALING
        combo_size = WIDTH*COMBOSCALING
        sampleList = []

        vbuffer_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        tool_sizer = wx.BoxSizer(wx.VERTICAL)
        text_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        
        # initialize rospack path manager
        self.rospack = rospkg.RosPack()
        path = self.rospack.get_path('ws_generator')
        path = os.path.join(path, 'src/utils/ref/')

        # backbutton image 
        image_file = os.path.join(path,"340.png")
        png = wx.Image(image_file, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        png = wx.Image(image_file)
        png = png.Scale(icon_size, icon_size, wx.IMAGE_QUALITY_HIGH)
        result = wx.BitmapFromImage(png)
        back_button = wx.BitmapButton(self, 1, result)
        
        # submit image 
        image_file = os.path.join(path,"submitBtn.png")
        png = wx.Image(image_file, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        png = wx.Image(image_file)
        png = png.Scale(icon_size, icon_size, wx.IMAGE_QUALITY_HIGH)
        
        result = wx.BitmapFromImage(png)
        self.submit_button = wx.BitmapButton(self, 1, result)
        self.submit_button.Disable() 
       
        # setup comboBox Lists
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

        textures = [OptionList(0, "Bump"),
                    OptionList(1, "Sinusoidal"),
                    OptionList(2, "Triangular"),
                    OptionList(3, "Square")]

        self.frequency = wx.ComboBox(self,
                              choices=['2','5','10'],
                              pos=(0.4*WIDTH,0.05*HEIGHT),
                              size=(combo_size,combo_size),
                              style=wx.CB_READONLY)
           
        
        self.texture = wx.ComboBox(self,
                              size=(combo_size,combo_size),
                              choices=sampleList,
                              pos=(0.4*WIDTH,0.05*HEIGHT),
                              style=wx.CB_READONLY) 
        self.widgetMaker(self.texture, textures)
        
        
        self.Amplitude_hybrid = wx.ComboBox(self,
                              size=(combo_size,combo_size),
                              choices=sampleList,
                              pos=(0.4*WIDTH,0.05*HEIGHT),
                              style=wx.CB_READONLY)
        self.widgetMaker(self.Amplitude_hybrid, amplitudes)
        
        # vertical spacer workaround for screen cover 
        s = " " 
        spacer_buffer_top = wx.StaticText(self, 1, s)
        font = wx.Font(VERTSPACERMULT, wx.ROMAN, wx.NORMAL, wx.NORMAL)
        spacer_buffer_top.SetFont(font)
        vbuffer_sizer.Add(spacer_buffer_top, 1, 0, 0)

        # Horizontal spacer workaround for screen cover 
        s = " " * HORIZSPACERSIZE 
        spacer_buffer_left = wx.StaticText(self, 1, s)
        spacer_buffer_right = wx.StaticText(self, 1, s)       

        button_sizer.Add(spacer_buffer_left,1,0,0)
        button_sizer.Add(back_button, 1, 0, 0) 
        button_sizer.Add(self.frequency, 2) 
        button_sizer.Add(self.texture, 2) 
        button_sizer.Add(self.Amplitude_hybrid, 2) 
        button_sizer.Add(self.submit_button, 1, 0, 0) 
        button_sizer.Add(spacer_buffer_right, 1, 0, 0)
        
        s = " " * (LEFTTEXTSPACING + TEXTOFFSETS[0]) + "Back"
        s += " " * (BUTTONTEXTSPACING - len("Back") + TEXTOFFSETS[1]) + "Frequency"
        s += " " * (COMBOTEXTSPACING - len("Frequency") + TEXTOFFSETS[2]) + "Texture"
        s += " " * (COMBOTEXTSPACING - len("Texture") + TEXTOFFSETS[3]) + "Hybrid"
        s += " " * (COMBOTEXTSPACING - len("Hybrid") + TEXTOFFSETS[4]) + "Submit"
        labels = wx.StaticText(self, 1, s) 
        
        font = wx.Font(HEIGHT*FONTSCALING, wx.ROMAN, wx.NORMAL, wx.NORMAL)
         
        #blank space is workaround for screen cover 
        labels.SetFont(font) 
        
        self.frequency.SetFont(font) 
        self.texture.SetFont(font) 
        self.Amplitude_hybrid.SetFont(font) 

        text_sizer.Add(labels, 1) 
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self.panel = DemoPanel(self, velocity, refresh, length)

        self.frequency.Bind(wx.EVT_TEXT, self.checkSelect)
        self.texture.Bind(wx.EVT_TEXT, self.checkSelect)
        self.Amplitude_hybrid.Bind(wx.EVT_TEXT, self.checkSelect)
        
        self.frequency.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.texture.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.Amplitude_hybrid.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
         
        back_button.Bind(wx.EVT_BUTTON, self.on_back_button) 
        self.submit_button.Bind(wx.EVT_BUTTON, self.generate_workspace) 
         
        tool_sizer.Add(vbuffer_sizer,0,wx.EXPAND)
        tool_sizer.Add(button_sizer,0,wx.EXPAND)
        tool_sizer.Add(text_sizer,0,wx.EXPAND)
        tool_sizer.Add(self.panel,10,wx.EXPAND) 

        #adds the sizers to a sizer to section of text, buttons, and panel display 
        self.SetSizer(tool_sizer)
         
        tool_sizer.Layout() 
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)

        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH

    def generate_workspace(self, evt):
        ws = {0: ["Hybrid", self.Amplitude_hybrid.GetValue(), self.texture.GetValue(), self.frequency.GetValue()]}
        
        ws_compress = 2
        intensity, y_ws = Generate_WS(self.panel, ws, ws_compress)
        # Account for sizers above demo panel
        y_ws += (self.HEIGHT - self.panel.HEIGHT)

        hybrid_msg = WSArray()
        hybrid_msg.header.stamp = rospy.Time(0.0)
        hybrid_msg.y_step = len(y_ws)
        hybrid_msg.y_ws = y_ws.flatten().tolist()
        hybrid_msg.int_compress = ws_compress
        hybrid_msg.intensity = intensity.flatten().tolist()

        #sends values once submit button is pressed
        self.ws_hybrid_pub.publish(hybrid_msg)


if __name__ == '__main__':
    app = wx.App(False)
    frame = DemoFrame(velocity=5, refresh=10, length=19)
    frame.Show(True)
    app.MainLoop()


