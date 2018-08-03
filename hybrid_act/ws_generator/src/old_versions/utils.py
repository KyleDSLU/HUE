#! /usr/bin/env python
import os
import time
import numpy as np

import wx

import main

class GuiFrame(wx.Frame):
    #----------------------------------------------------------------------
    def __init__(self):
        wx.Frame.__init__(self, parent = None, id = wx.ID_ANY, title = wx.EmptyString,size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU)

        self.width, self.height = wx.GetDisplaySize()
        self.width_half = self.width/2.0

        self.FIRST_RECTANGLE_Y = 0.1*self.height
        self.BOTTOM_SPACE = 0.035*self.height

        self.RECTANGLE_SIZE = 0.175*self.height
        self.RECTANGLE_SEPERATION = int((self.height-self.FIRST_RECTANGLE_Y-self.BOTTOM_SPACE-3*self.RECTANGLE_SIZE)/3.0)

        #set up textbox and lines
        self.TEXTBOX_WIDTH = self.width
        self.TEXTBOX_X = 0.04*self.width
        self.TEXTBOX_Y = self.RECTANGLE_SIZE*0.35
        self.RECTANGLE_COLOR = "WHITE"
        self.textbox_fontsize = int(42*((self.width*self.height)/(1794816.)))

        self.HAPTIC_WIDTH = self.width

        self.BACKGROUND_COLOR = "BLACK"

        # Add a panel so it looks correct on all platforms
        self.panel = wx.Panel(self, wx.ID_ANY)
        self.panel.Bind(wx.EVT_PAINT, self.OnPaint)

    def layout(self):#set up buttons and texts
        if self.tc[0] == 1:
            gui_question = "Which Texture Felt Stronger?"

        back_button = wx.Button(self.panel,wx.ID_ANY,'BACK')
        label = wx.StaticText(self.panel,wx.ID_ANY,label=gui_question,pos=(0,.7*self.height))
        label.SetFont(wx.Font(self.textbox_fontsize,wx.ROMAN,wx.NORMAL,wx.BOLD))
        button_1 = wx.Button(self.panel,wx.ID_ANY, '1',pos=(0,self.height*.8),size=(self.width_half,.2*self.height))
        button_2 = wx.Button(self.panel,wx.ID_ANY, '2',pos=(self.width_half,self.height*.8),size=(self.width_half,.2*self.height))

        #set up font
        button_1.SetFont(wx.Font(self.textbox_fontsize,wx.ROMAN,wx.NORMAL,wx.BOLD))
        button_2.SetFont(wx.Font(self.textbox_fontsize,wx.ROMAN,wx.NORMAL,wx.BOLD))

        ##connect buttons to functions
        back_button.Bind(wx.EVT_BUTTON,self.BackButton)
        #add arguments when connecting buttons to functions using lambda
        button_1.Bind(wx.EVT_BUTTON,lambda evt: self.option(evt,1))
        button_2.Bind(wx.EVT_BUTTON,lambda evt: self.option(evt,2))
        #self.OnPaint(0)#draw again

    #go back
    def BackButton(self,event):
        f = main.frameMain(None)
        self.Close()
        f.Show()

    #----------------------------------------------------------------------
    def OnPaint(self, evt):
        """set up the device for painting"""
        dc = wx.PaintDC(self.panel)
        dc.BeginDrawing()
        brush = wx.Brush(self.BACKGROUND_COLOR)
        dc.SetBackground(brush)
        dc.Clear()
        dc.SetFont(wx.Font(self.textbox_fontsize, wx.ROMAN, wx.FONTSTYLE_NORMAL, wx.NORMAL))

        """Rectangle 1"""
        RECTANGLE_Y = self.FIRST_RECTANGLE_Y
        dc.SetPen(wx.Pen(self.RECTANGLE_COLOR))
        dc.SetBrush(wx.Brush(self.RECTANGLE_COLOR))
        dc.DrawRectangle(0,RECTANGLE_Y, self.width, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X, RECTANGLE_Y+self.TEXTBOX_Y)
        dc.DrawLabel("1)", textbox, alignment=1)

        """Rectangle 2"""
        RECTANGLE_Y = self.FIRST_RECTANGLE_Y+(self.RECTANGLE_SIZE+self.RECTANGLE_SEPERATION)
        dc.DrawRectangle(0,RECTANGLE_Y, self.width, self.RECTANGLE_SIZE)
        textbox = wx.Rect(self.TEXTBOX_X, RECTANGLE_Y+self.TEXTBOX_Y)
        dc.DrawLabel("2)", textbox, alignment=1)
        dc.EndDrawing()
        del dc

    def generate_ws(self):
        intensity = np.zeros([4,self.HAPTIC_WIDTH])
        """Determine y workspace bounds"""
        y_ws = np.zeros([2,2])
        RECTANGLE_Y = self.FIRST_RECTANGLE_Y
        y_ws[0] = [RECTANGLE_Y,RECTANGLE_Y+self.RECTANGLE_SIZE]
        RECTANGLE_Y = RECTANGLE_Y + self.RECTANGLE_SIZE + self.RECTANGLE_SEPERATION
        y_ws[1] = [RECTANGLE_Y,RECTANGLE_Y + self.RECTANGLE_SIZE]

        HAPTIC_WIDTH = float(self.HAPTIC_WIDTH)
        horiz_pixels = np.arange(HAPTIC_WIDTH)

        # output has form of channel: actuation, amplitude, texture, frequency
        for i,value in enumerate(self.rand_output.values()):
            amp = int(value[1]*100)
            if value[2] == "Sinusoid":
                if value[0] == "Hybrid":
                    sinusoid = amp*np.sin(horiz_pixels/HAPTIC_WIDTH*value[3]*2*np.pi)
                    ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]
                    intensity[2*i][ind[0]] = sinusoid[ind[0]]
                    intensity[2*i+1][ind[1]] = -sinusoid[ind[1]]
                elif value[0] == "UFM":
                    intensity[2*i] = amp/2*np.sin(horiz_pixels/HAPTIC_WIDTH*value[3]*2*np.pi) + amp/2
                elif value[0] == "EV":
                    intensity[2*i+1] = amp/2*np.sin(horiz_pixels/HAPTIC_WIDTH*value[3]*2*np.pi) + amp/2

            elif value[2]  == "Square":
                sinusoid = np.sin(horiz_pixels/HAPTIC_WIDTH*value[3]*2*np.pi)
                ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]

                if value[0] == "Hybrid":
                    intensity[2*i][ind[0]] = amp
                    intensity[2*i+1][ind[1]] = amp
                elif value[0] == "UFM":
                    intensity[2*i][ind[0]] = amp
                elif value[0] == "EV":
                    intensity[2*i+1][ind[1]] = amp

        self.start_time = time.time()
        return intensity.astype(int).tolist(), y_ws.astype(int).tolist()

