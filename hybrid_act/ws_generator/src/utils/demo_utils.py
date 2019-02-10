#! /usr/bin/env python
import os
import time 
import numpy as np 
from ws_generator.msg import WSArray, IntArray, ForceChannel, ForceArray
import time
import rospkg
import rospy
import wx
import main
from std_msgs.msg import Bool
from utils import Ball, Generate_WS, NormForcePanel, NormForceLabel

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

        #create cursor position subscriber
        self.ir_x = 0
        self.ir_y = 0
        
	self.update_drawing()
        
        self.ir_sub = rospy.Subscriber('/cursor_position/corrected', IntArray, self.cursor_callback, queue_size = 1)
	
        
	self.force_channel = [ForceChannel()]
        self.force_channel[0].channels = 1 
        self.force_channel[0].measure_channel = 0

        self.forcechannel_pub = rospy.Publisher('/force_recording/force_channel', ForceChannel, queue_size = 0)
        self.master_force_pub = rospy.Publisher('/hue_master/force', Bool, queue_size = 0)
        self.master_actuation_pub = rospy.Publisher('/hue_master/actuation', Bool, queue_size = 0)
	
	wx.CallLater(200, self.SetFocus)

    def on_size(self, event):
        self.WIDTH, self.HEIGHT = self.GetClientSize()
        self.CURSOR_OFFSET = self.parent.HEIGHT - self.HEIGHT

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

    def update_drawing(self, *args):
        self.on_update(self.ir_x, self.ir_y)

    def cursor_callback(self, ir_xy):
        self.ir_x = ir_xy.data[0]
        self.ir_y = ir_xy.data[1] - self.CURSOR_OFFSET

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


class DemoFrame(wx.Frame):
    def __init__(self, velocity, refresh, length, *args, **kw):
        self._layout(velocity, refresh, length)
        self.time = wx.DateTime.Now()
    	self.path = self.rospack.get_path('ws_generator') 
        self.SEPERATION_CHAR = -999
        self.lengths = [0,0]
        self.tan_force = []
        self.norm_force = []
        self.x_positions = []
        self.intensity = []

        self.force_sub = rospy.Subscriber('/force_recording/force_records', ForceArray, self.force_callback, queue_size = 1)
    
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

        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH

        FONTSCALING = 0.01
        ICONSCALING = 0.01
        COMBOSCALING = ICONSCALING*1.1
        HORIZSPACERSIZE = 4
        VERTSPACERMULT = HEIGHT*0.01

        LEFTTEXTSPACING = int(WIDTH*0.008)
        BUTTONTEXTSPACING = int(WIDTH*0.007)
        COMBOTEXTSPACING = int(WIDTH*0.0135)
        TEXTOFFSETS = [0,0,0,3,2]

        NORM_FORCE_DESIRED = 100
        NORM_FORCE_THRESHOLD = 20

        icon_size = WIDTH*ICONSCALING
        combo_size = WIDTH*COMBOSCALING
        sampleList = []

        vbuffer_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        tool_sizer = wx.BoxSizer(wx.VERTICAL)
       	text_norm_hsizer = wx.BoxSizer(wx.HORIZONTAL) 	
       	norm_vsizer = wx.BoxSizer(wx.VERTICAL) 
        csv_vsizer = wx.BoxSizer(wx.VERTICAL)	
	text_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        norm_label_sizer = wx.BoxSizer(wx.HORIZONTAL)
        norm_force_sizer = wx.BoxSizer(wx.HORIZONTAL)
        save_vsizer = wx.BoxSizer(wx.VERTICAL)	
	        
        # Horizontal spacer workaround for screen cover 
        s = " " * HORIZSPACERSIZE 
        spacer_buffer_left = wx.StaticText(self, 1, s)
        spacer_buffer_right = wx.StaticText(self, 1, s)       
        s = " " * int(HORIZSPACERSIZE/3)
        spacer_buffer_middle1 = wx.StaticText(self, 1, s)
        
        spacer_buffer_middle2 = wx.StaticText(self, 1, s)
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

        #Add Buttons and comboboxes
        button_sizer.Add(spacer_buffer_left,1,0,0)
        button_sizer.Add(back_button, 1, 0, 0) 
        button_sizer.Add(self.frequency, 2) 
        button_sizer.Add(self.texture, 2) 
        button_sizer.Add(self.Amplitude_hybrid, 2) 
        button_sizer.Add(self.submit_button, 1, 0, 0) 
        button_sizer.Add(spacer_buffer_right, 1, 0, 0)
        
        #Setup font
        font = wx.Font(HEIGHT*FONTSCALING, wx.ROMAN, wx.NORMAL, wx.NORMAL)

        #Create button/cb labels
        #blank space is workaround for screen cover 
        s = " " * (LEFTTEXTSPACING + TEXTOFFSETS[0]) + "Back"
        s += " " * (BUTTONTEXTSPACING - len("Back") + TEXTOFFSETS[1]) + "Frequency"
        s += " " * (COMBOTEXTSPACING - len("Frequency") + TEXTOFFSETS[2]) + "Texture"
        s += " " * (COMBOTEXTSPACING - len("Texture") + TEXTOFFSETS[3]) + "Hybrid"
        s += " " * (COMBOTEXTSPACING - len("Hybrid") + TEXTOFFSETS[4]) + "Submit"
        labels = wx.StaticText(self, 1, s) 
        labels.SetFont(font) 
        text_sizer.Add(labels, 1) 
        
        #Set combobox font
        self.frequency.SetFont(font) 
        self.texture.SetFont(font) 
        self.Amplitude_hybrid.SetFont(font) 

        #Setup comboboxes
        self.frequency.Bind(wx.EVT_TEXT, self.checkSelect)
        self.texture.Bind(wx.EVT_TEXT, self.checkSelect)
        self.Amplitude_hybrid.Bind(wx.EVT_TEXT, self.checkSelect)
        
        self.frequency.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.texture.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
        self.Amplitude_hybrid.Bind(wx.EVT_COMBOBOX_CLOSEUP, self.update_panel)
         
        #Setup back and submit buttons
        back_button.Bind(wx.EVT_BUTTON, self.on_back_button) 
        self.submit_button.Bind(wx.EVT_BUTTON, self.generate_workspace) 
         
        #Add sizers to overall sizer
        tool_sizer.Add(vbuffer_sizer, 0, wx.EXPAND)
        tool_sizer.Add(button_sizer, 0, wx.EXPAND)
        tool_sizer.Add(text_sizer, 0, wx.EXPAND)

	#Add sizer for .csv name input
	csv_name_label = wx.StaticText(self, id = -1, label = "csv file", style = wx.ALIGN_CENTER,  name = "csv file") 
	
	self.csv_text_box = wx.TextCtrl(self, size = (200,50)) 
        csv_vsizer.Add(self.csv_text_box, 0, wx.EXPAND)
	csv_vsizer.Add(csv_name_label, 0, wx.EXPAND) 
	#Add sizer for normal force
        
        norm_force_label = NormForceLabel(self, HEIGHT*FONTSCALING)
        norm_vsizer.Add(norm_force_label, 0, wx.EXPAND)
        norm_force_panel = NormForcePanel(self, HEIGHT*FONTSCALING, NORM_FORCE_DESIRED, NORM_FORCE_THRESHOLD)
        norm_vsizer.Add(norm_force_panel, 0, wx.EXPAND)
         		

	#Add button sizer components
		
        button_size = wx.Size(int(HEIGHT*0.01), WIDTH*0.05)
        save_button = wx.Button(self, wx.ID_ANY, 'SAVE', size = button_size)
	
        save_button.Bind(wx.EVT_BUTTON, self.on_save_button) 
	#save_button_label = wx.StaticText(self, id = -1, label = "Save", style = wx.ALIGN_CENTER,  name = "Save") 
	save_vsizer.Add(save_button, 0, wx.EXPAND)
	#save_vsizer.Add(save_button_label, 0, wx.EXPAND)
	
	#Add components to hsizer 
      
        text_norm_hsizer.Add(spacer_buffer_left, 1, wx.LEFT)
        text_norm_hsizer.Add(csv_vsizer, 1, wx.CENTER)	
	text_norm_hsizer.Add(spacer_buffer_middle1, 1, wx.EXPAND)
	text_norm_hsizer.Add(save_vsizer,1 ,wx.CENTER)
	text_norm_hsizer.Add(spacer_buffer_middle2, 1, wx.EXPAND)
        text_norm_hsizer.Add(norm_vsizer, 1, wx.EXPAND)
        text_norm_hsizer.Add(spacer_buffer_right, 1, wx.RIGHT)
	



	tool_sizer.Add(text_norm_hsizer, 0) 
	 
	#Create Ball sizer
        self.panel = DemoPanel(self, velocity, refresh, length)
        tool_sizer.Add(self.panel, 10, wx.EXPAND) 

        #adds the sizers to section of text, buttons, and panel display 
        self.SetSizer(tool_sizer)
         
        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)

        tool_sizer.Layout() 
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)
    
    def force_callback(self, force_array):
        self.lengths = force_array.lengths
        self.tan_force = force_array.tan_force
        self.norm_force = force_array.norm_force
        self.x_positions = force_array.x_positions
        self.intensity = force_array.intensity
        print("here", self.lengths)
    
    def on_save_button(self, *args):    	
    	#save on the csv file the name of the user
    	self.csvfile = self.path + '/src/csvfiles/' + self.csv_text_box.GetValue() + '.csv'
	with open(self.csvfile, 'a+') as fout:
            index = 0
	    l = []
            for length in self.lengths:
                l.extend(self.tan_force[index:length+index])
                l.append(self.SEPERATION_CHAR)
                l.extend(self.norm_force[index:length+index])
                l.append(self.SEPERATION_CHAR)
                l.extend(self.intensity[index:length+index])
                l.append(self.SEPERATION_CHAR)
                index = length

            l = [str(i) for i in l]
            s = ','.join(l) + "\n"
            print(s)
            fout.write(s)
            fout.close()

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


