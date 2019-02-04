#! /usr/bin/env python
import random
import time

import wx
import os

import rospy
import rospkg
from ws_generator.msg import WSArray, ForceArray
from std_msgs.msg import String, Bool

#Other GUI utilites
import main
import utils.start_utils as start_utils

from utils.utils import Ball, Generate_WS, NormForcePanel, NormForceLabel, QuestionPanel

class Frame(wx.Frame):
    #----------------------------------------------------------------------
    def __init__(self):
        """"""
        rospy.init_node('demo_ws')
        self.ws_hybrid_pub = rospy.Publisher('/cursor_position/workspace/hybrid', WSArray, queue_size = 0)
        self.master_force_pub = rospy.Publisher('/hue_master/force', Bool, queue_size = 0)
        self.master_actuation_pub = rospy.Publisher('/hue_master/actuation', Bool, queue_size = 0)
        self.force_sub = rospy.Subscriber('/cursor_position/force/force_list', ForceArray, self.force_callback, queue_size = 1)

        b = Bool()
        b.data = True
        self.master_force_pub.publish(b)
        self.master_actuation_pub.publish(b)

        self.REFRESH_RATE = 50
        self.SCREEN_LENGTH = 15
        self.BALL_VELOCITY = 10     #cm/s

        self._layout(self.BALL_VELOCITY, self.REFRESH_RATE, self.SCREEN_LENGTH)
        self.time = wx.DateTime.Now()

        # variables for the amount of times each testing condition is finished
        self.AMPLITUDE_COUNT = 0

        self.CORRECT = None
        self.THRESHOLD_FLIPS = 0   # variable to save the amount of times user has guessed wrong after guessing right
        self.SIG_THRESHOLDS = 1
        self.REP_INCORRECT = 0

        self.REPEAT_TESTS = 1   #variable to determine repeat of same tests
        self.TEST_CASE = 0      #variable to iterate through tests of each testing condition

        self.AMPLITUDE_MAX = 1.0
        self.AMPLITUDE_MIN = 0.75
        self.DELTA_AMPLITUDE = 0.05

        self.ws_output = None
        self.rand_output = None

        self.test_conditions = None

        # Generate Gui
        self.Centre()
        self.Show()

    def determine_next_test(self):
        # start hybridization test
        if self.AMPLITUDE_COUNT < self.REPEAT_TESTS:
            self.amplitude_set()
            self.tc = self.test_conditions[0]
            self.AMPLITUDE_COUNT += 1
        else:
            f = main.frameMain(None)
            self.Close()
            f.Show()

    def determine_next_condition(self):
        if self.THRESHOLD_FLIPS < self.SIG_THRESHOLDS or self.FINISH_FLAG:
            if not self.ws_output:
                # Construct output in the form of, channel: actuation, amplitude, texture, frequency
                if self.tc[0] == 'AMPLITUDE_TEST':
                    self.ws_output = {0: [self.tc[1], self.AMPLITUDE_MIN, self.tc[3], self.tc[4]], \
                                      1: [self.tc[2], 1.0, self.tc[3], self.tc[4]]}
            if self.CORRECT == True:
                # increase amplitude of test condition to make test harder
                self.ws_output[0][1] += self.DELTA_AMPLITUDE

            elif self.CORRECT == False:
                # decrease amplitude of test condition to make test easier
                self.ws_output[0][1] = min(self.AMPLITUDE_MIN,self.ws_output[0][1]-2*self.DELTA_AMPLITUDE)

            self.randomize_output()
            self.define_correct_selection()
            print(self.rand_output)
            intensity, y_ws = self.generate_workspace(self.rand_output)
            self.publish_intensity(intensity,y_ws)

        else:
            # reset Threshold flips
            self.THRESHOLD_FLIPS = 0
            self.FINISH_FLAG = False
            # remove last test case from possible test_cases
            del self.test_conditions[self.TEST_CASE]
            self.TEST_CASE += 1
            self.ws_output = None
            self.rand_output = None
            self.CORRECT = None
            try:
                self.tc = self.test_conditions[self.TEST_CASE]
                self.determine_next_condition()
            except KeyError as e:
                # Fall in here if self.test_conditions is empty
                self.TEST_CASE = 0
                self.determine_next_test()

    def amplitude_set(self):
        # construct conditions in the form of, test#: test_id, test_actuation, control_actuation, texture, freq
        self.test_conditions = {0:['AMPLITUDE_TEST',"Hybrid","Hybrid","Sinusoid",2], \
                                1:['AMPLITUDE_TEST',"Hybrid","Hybrid","Sinusoid",5], \
                                2:['AMPLITUDE_TEST',"Hybrid","Hybrid","Sinusoid",10]}

    def randomize_output(self):
        # randomize channel 0 and 1
        key1,key2 = random.sample(list(self.ws_output),2)
        self.rand_output = {}
        self.rand_output[key1], self.rand_output[key2] = self.ws_output[0], self.ws_output[1]


    def define_correct_selection(self):
        # determine which output channel is the correct choice
        if self.rand_output[0][1] == self.tc[1]:
            self.correct_selection = 0
        else:
            self.correct_selection = 1

    def define_correctness(self,selected):
        if selected == self.correct_selection:
            if not self.CORRECT:
                self.CORRECT = True
            if self.ws_output[0][1] >= self.AMPLITUDE_MAX:
                self.FINISH_FLAG = True

            self.REP_INCORRECT = 0
        else:
            if self.CORRECT:
                self.THRESHOLD_FLIPS += 1
                self.CORRECT = False

            self.REP_INCORRECT += 1

    def force_callback(self, force_array):
        pass

    def close(self):
        self.Close()
        f = main.frameMain(None)
        f.show()

    ###########################################################################################################

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

    def on_submit_button(self, event):
        self.submit_button.Disable() 

        self.determine_next_test()
        self.determine_next_condition()

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

    def generate_workspace(self, ws):
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

    def _layout(self, velocity, refresh, length):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = "Haptic Demo", size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)
        WIDTH, HEIGHT = self.GetClientSize()
        #Save parameters for later calculations
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

        NORM_FORCE_DESIRED = 100
        NORM_FORCE_THRESHOLD = 20
        button_size = wx.Size(int(HEIGHT*0.01), WIDTH*0.05)

        icon_size = WIDTH*ICONSCALING
        combo_size = WIDTH*COMBOSCALING
        sampleList = []

        vbuffer_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        tool_sizer = wx.BoxSizer(wx.VERTICAL)
        text_sizer = wx.BoxSizer(wx.HORIZONTAL)  
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        norm_label_sizer = wx.BoxSizer(wx.HORIZONTAL)
        norm_force_sizer = wx.BoxSizer(wx.HORIZONTAL)
        
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
        back_button = wx.BitmapButton(self, 1, result, size = button_size)
        
        # submit image 
        image_file = os.path.join(path,"submitBtn.png")
        png = wx.Image(image_file, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
        png = wx.Image(image_file)
        png = png.Scale(icon_size, icon_size, wx.IMAGE_QUALITY_HIGH)
        
        result = wx.BitmapFromImage(png)
        self.submit_button = wx.BitmapButton(self, 1, result, size = button_size)

        # 1 and 2 Buttons
        button_1 = wx.Button(self, wx.ID_ANY, '1', size = button_size)
        button_2 = wx.Button(self, wx.ID_ANY, '2', size = button_size)
       
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
        s = " " * int(HORIZSPACERSIZE/3)
        spacer_buffer_middle = wx.StaticText(self, 1, s)

        #Add Buttons and comboboxes
        button_sizer.Add(spacer_buffer_left,1,0,0)
        button_sizer.Add(back_button, 1, 0, 0) 
        button_sizer.Add(self.submit_button, 1, 0, 0) 
        button_sizer.Add(spacer_buffer_middle, 1, 0, 0)
        button_sizer.Add(button_1, 1, 0, 0)
        button_sizer.Add(button_2, 1, 0, 0)
        button_sizer.Add(spacer_buffer_right, 1, 0, 0)
        
        #Setup font
        font = wx.Font(HEIGHT*FONTSCALING, wx.ROMAN, wx.NORMAL, wx.NORMAL)

        #Setup back and submit buttons
        back_button.Bind(wx.EVT_BUTTON, self.on_back_button) 
        self.submit_button.Bind(wx.EVT_BUTTON, self.on_submit_button) 
         
        #Add sizers to overall sizer
        tool_sizer.Add(vbuffer_sizer, 0, wx.EXPAND)
        tool_sizer.Add(button_sizer, 0, wx.EXPAND)
        tool_sizer.Add(text_sizer, 0, wx.EXPAND)

        #Add sizer for normal force and question
        overall_sizer = wx.BoxSizer(wx.HORIZONTAL)
        vsizer = wx.BoxSizer(wx.VERTICAL)
        hsizer = wx.BoxSizer(wx.HORIZONTAL)
        norm_force_label = NormForceLabel(self, HEIGHT*FONTSCALING)
        norm_force_panel = NormForcePanel(self, HEIGHT*FONTSCALING, NORM_FORCE_DESIRED, NORM_FORCE_THRESHOLD)
        question_panel = QuestionPanel(self, HEIGHT*FONTSCALING)
        vsizer.Add(norm_force_label,0, wx.EXPAND)
        vsizer.Add(norm_force_panel, 0, wx.EXPAND)

        hsizer.Add(spacer_buffer_left, 1, wx.LEFT)
        hsizer.Add(question_panel, 1, wx.EXPAND)
        hsizer.Add(spacer_buffer_middle, 1, wx.EXPAND)
        hsizer.Add(vsizer, 1, wx.EXPAND)
        hsizer.Add(spacer_buffer_right, 1, wx.RIGHT)
        overall_sizer.Add(hsizer, 1, wx.EXPAND)
        tool_sizer.Add(overall_sizer, 1, wx.EXPAND)

        #Create Ball sizer
        self.panel = start_utils.DemoPanel(self, velocity, refresh, length)
        tool_sizer.Add(self.panel, 10, wx.EXPAND) 

        #adds the sizers to section of text, buttons, and panel display 
        self.SetSizer(tool_sizer)
        tool_sizer.Layout() 

        self.Bind(wx.EVT_CLOSE, self.on_close)

        #Create a timer to handle gui update
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self.timer = wx.Timer(self)
        self.timer.Start(refresh)




# Run the program
if __name__ == "__main__":
    app = wx.App(False)
    frame = Frame()
    frame.Show()
    app.MainLoop()
