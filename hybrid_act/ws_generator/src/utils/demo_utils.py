#! /usr/bin/env python
import os
import time 
import numpy as np 
from ws_generator.msg import WSArray
import time
import rospkg
import wx
import main

from utils import Ball, ws_generator

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

        self.ball = [[],[],[]]
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
        self.wait_count = [0]*len(self.ball)

        self.on_size(0)
        self.update_drawing()
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
        third_ball_start = int(second_ball_start+self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        self.BALL_START = [[self.BALL_LEFTMARGIN,first_ball_start],[self.BALL_LEFTMARGIN,second_ball_start],[self.BALL_LEFTMARGIN,third_ball_start]]

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
                try:
                    self.import_bitmap(path)
                    print 'Importing bitmap' 
                    self.BITMAP_FLAG = 1

                except:
                    print 'Couldn\'t find it, give me one second'
                    self.generate_png(dc,path)
                    time.sleep(0.01)
                    self.import_bitmap(path)
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
               print "hello"

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

    def import_bitmap(self, path):
        img  = path + 'demo_'+str(self.WIDTH)+'_'+str(self.HEIGHT)+'.png'
        try:
            del self.bmp
            del self.bitmap
        except:
            pass
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
        self.odc.Clear()
        redraw_list = [True] * len(self.ball)
        pos_list = self.BALL_START
        self.draw_balls(redraw_list, pos_list)

        #clears panel when frame combo boxes are rolled up.


class DemoFrame(wx.Frame):
    def __init__(self, velocity, refresh, length, *args, **kw):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = "Haptic Demo", size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)
        self.WIDTH = 100
        self.HEIGHT = 100
        iconSize = (24, 24)
        icon = 24
        comboSize =40 
        sampleList = []
        self.rospack = rospkg.RosPack()
            
        self.toolSizer = wx.BoxSizer(wx.VERTICAL)
        self.textSizer = wx.BoxSizer(wx.HORIZONTAL)  
        self.buttonSizer = wx.BoxSizer(wx.HORIZONTAL)
        
        
        path = self.rospack.get_path('ws_generator')
        path = os.path.join(path, 'src/utils/ref/')

        imageFile = os.path.join(path,"340.png")
        #backbutton image 
        png = wx.Image(imageFile, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        png = wx.Image(imageFile)
        png = png.Scale(icon,icon, wx.IMAGE_QUALITY_HIGH)
        result = wx.BitmapFromImage(png)
        
        imageFile = os.path.join(path,"submitBtn.png")
        png = wx.Image(imageFile, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        png = wx.Image(imageFile)
        png = png.Scale(icon,icon, wx.IMAGE_QUALITY_HIGH)
        
        result1 = wx.BitmapFromImage(png)
       
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
        #setup comboBox Lists


        self.backButton = wx.BitmapButton(self, 1, result)

        self.submitButton = wx.BitmapButton(self, 1, result1)
        self.submitButton.Disable() 
        self.frequency = wx.ComboBox(self,
                              choices=['2','5','10'],
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT),
                              size=(comboSize,comboSize),
                              style=wx.CB_READONLY)
           
        
        self.texture = wx.ComboBox(self,
                              size=(comboSize,comboSize),
                              choices=sampleList,
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT),
                              style=wx.CB_READONLY) 
        
        self.widgetMaker(self.texture, textures)
        
        
        self.Amplitude_EV = wx.ComboBox(self,
                              size=(comboSize,comboSize),
                              choices=sampleList,
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT),
                              style=wx.CB_READONLY)
        
        self.widgetMaker(self.Amplitude_EV, amplitudes)
         
        self.Amplitude_UFM = wx.ComboBox(self,
                              size=(comboSize,comboSize),
                              choices=sampleList,
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT),
                              style=wx.CB_READONLY)
        
        self.widgetMaker(self.Amplitude_UFM, amplitudes)
        
        self.Amplitude_hybrid = wx.ComboBox(self,
                              size=(comboSize,comboSize),
                              choices=sampleList,
                              pos=(0.4*self.WIDTH,0.05*self.HEIGHT),
                              style=wx.CB_READONLY)
        self.widgetMaker(self.Amplitude_hybrid, amplitudes)
        
        self.spacerBuffer1 = wx.StaticText(self, 1, "   ")       
        self.spacerBuffer2 = wx.StaticText(self, 1, "   ")       
        #spacer workaround for screen cover 
        self.buttonSizer.Add(self.spacerBuffer1,1,0,0)
        self.buttonSizer.Add(self.backButton,1,0,0 ) 

        self.buttonSizer.Add(self.frequency,2) 
       
        self.buttonSizer.Add(self.texture,2 ) 
        self.buttonSizer.Add(self.Amplitude_EV, 2) 
        self.buttonSizer.Add(self.Amplitude_UFM, 2) 
        self.buttonSizer.Add(self.Amplitude_hybrid, 2) 
         
        self.buttonSizer.Add(self.submitButton,1,0,0 ) 
        self.buttonSizer.Add(self.spacerBuffer2,1,0,0)
        
        
        
        self.backLabel = wx.StaticText(self, 1, "                   Back")
        self.submitLabel = wx.StaticText(self,1 , "          Submit                   ") 
        self.freqLabel = wx.StaticText(self, 1, "              Frequency")
        self.textureLabel = wx.StaticText(self, 1,"          Texture",size = (comboSize, comboSize))
        self.evLabel = wx.StaticText(self, 1, "          EV")
        self.ufmLabel = wx.StaticText(self, 1, "          UFM")
        self.hybridLabel = wx.StaticText(self, 1, "          Hybrid")
        
        font = wx.Font(18, wx.ROMAN, wx.NORMAL, wx.NORMAL)
         
        #blank space is workaround for screen cover 
        
        self.backLabel.SetFont(font) 
        self.submitLabel.SetFont(font) 
        self.freqLabel.SetFont(font)  
        self.textureLabel.SetFont(font) 
        self.evLabel.SetFont(font)  
        self.ufmLabel.SetFont(font) 
        self.hybridLabel.SetFont(font) 
        
        self.frequency.SetFont(font) 
        self.texture.SetFont(font) 
        self.Amplitude_EV.SetFont(font) 
        self.Amplitude_UFM.SetFont(font) 
        self.Amplitude_hybrid.SetFont(font) 

        self.textSizer.Add(self.backLabel, 1)
        self.textSizer.Add(self.freqLabel,3)
        self.textSizer.Add(self.textureLabel, 3)
        self.textSizer.Add(self.evLabel, 3)
        self.textSizer.Add(self.ufmLabel, 3)  
        self.textSizer.Add(self.hybridLabel, 3)  
        self.textSizer.Add(self.submitLabel, 1) 
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self.panel = DemoPanel(self, velocity, refresh, length)
        self.ws_gen = ws_generator(self.panel) 

        self.frequency.Bind(wx.EVT_TEXT, self.checkSelect)
        self.texture.Bind(wx.EVT_TEXT, self.checkSelect)
        self.Amplitude_EV.Bind(wx.EVT_TEXT, self.checkSelect)
        self.Amplitude_UFM.Bind(wx.EVT_TEXT, self.checkSelect)
        self.Amplitude_hybrid.Bind(wx.EVT_TEXT, self.checkSelect)
        
        self.frequency.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.texture.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.Amplitude_EV.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.Amplitude_UFM.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.Amplitude_hybrid.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
         
        self.backButton.Bind(wx.EVT_BUTTON, self.back_button) 
        self.submitButton.Bind(wx.EVT_BUTTON, self.generate_workspace) 
         
        self.toolSizer.Add(self.buttonSizer,0,wx.EXPAND)
        self.toolSizer.Add(self.textSizer,0,wx.EXPAND)
        self.toolSizer.Add(self.panel,10,wx.EXPAND) 
        #adds the sizers to a sizer to section of text, buttons, and panel display 
        self.selectFlag = False
        self.SetSizer(self.toolSizer)
         
        self.toolSizer.Layout() 
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)


    def generate_workspace(self,evt):
        ws = {0: ["EV", self.Amplitude_EV.GetValue(), self.texture.GetValue(), self.frequency.GetValue()],
              1: ["UFM", self.Amplitude_UFM.GetValue(), self.texture.GetValue(), self.frequency.GetValue()],
              2: ["Hybrid", self.Amplitude_hybrid.GetValue(), self.texture.GetValue(), self.frequency.GetValue()]}
        
        intensity, y_ws = self.ws_gen.generate_ws(ws)

        ufm_msg = WSArray()
        ufm_msg.header.stamp = rospy.Time(0.0)
        ufm_msg.y_step = len(y_ws)
        ufm_msg.y_ws = y_ws[(0,2,4),:].flatten().tolist()
        ufm_msg.int_step = len(intensity)
        ufm_msg.intensity = intensity[(1,3,5),:].flatten().tolist()

        ev_msg = WSArray()
        ev_msg.header.stamp = rospy.Time(0.0)
        ev_msg.y_step = len(y_ws)
        ev_msg.y_ws = y_ws[(1,3,5),:].flatten().tolist()
        ev_msg.int_step = len(intensity)
        ev_msg.intensity = intensity[(1,3,5),:].flatten().tolist()

        self.parent.ws_ufm_pub.publish(ufm_msg)
        self.parent.ws_ev_pub.publish(ev_msg)
        #sends values once submit button is pressed


    def update_panel(self,evt):
        try:
            self.panel.updateFlag = True  
        except AttributeError:
            pass 
            #catching error if panel attribute does not exist i
     #checks if the panel needs updated after the frame dropdowns have covered it
    def back_button(self,event):
        f = main.frameMain(None)
        self.Close()
        f.Show()
    #toggles main screen and demo panel 
    def checkSelect(self, evt):
        if not ((self.frequency.GetValue() == '') or (self.texture.GetValue() == '') or (self.Amplitude_EV.GetValue() == '') or (self.Amplitude_UFM.GetValue() == '') or (self.Amplitude_hybrid.GetValue() == '')):
            self.submitButton.Enable()            
    #checks to see if all selections have been made before submit button is enabled
    def widgetMaker(self, widget, objects):
        """"""
        for obj in objects:
            widget.Append(obj.shape, obj)
    #helps with makeing combobox lists


    def on_close(self, event):
        self.timer.Stop()
        self.Destroy()

    def on_timer(self, event):
        self.panel.update_drawing()

if __name__ == '__main__':
    app = wx.App(False)
    frame = DemoFrame(velocity=5, refresh=10, length=19)
    frame.Show(True)
    app.MainLoop()


