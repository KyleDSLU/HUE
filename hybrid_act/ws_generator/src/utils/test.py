#! /usr/bin/env python
import os
import time
import numpy as np

from ws_generator.msg import WSArray

import rospkg
import wx
import main


class DemoPanel(wx.Panel):
    """Class DemoPanel creates a panel with the background inmage on it, inherits wx.Panel
    """
    def __init__(self, parent, id):
        wx.Panel.__init__(self, parent, id)
        #self.parent = parent
        try:
            #self.ball = [[],[],[]]
            #self.rospack = rospkg.RosPack()
            #self.last_pos = self.ScreenToClient(wx.GetMousePosition())
        #self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        #self.SetBackgroundColour("BLACK") 
        #self.Bind(wx.EVT_PAINT, self.on_paint)
        #self.Bind(wx.EVT_SIZE, self.on_size)
        #self.REFRESH = refresh
        #self.LENGTH = length
        #self.BALL_VELOCITY = velocity
        
        #self.imageCtrl = wx.StaticBitmap(self.Panel, wx,ID_ANY, wx,EmptyBitmap(517, 524))
        #path = self.rospack.get_path('ws_generator')
        #path = os.path.join(path, 'src/utils/saved.png')

            image_file = 'saved.png'
            bmp1 = wx.Image(image_file, wx.BITMAP_TYPE_ANY).ConvertToBitmap()
            self.bitmap1 = wx.StaticBitmap(self, -1, bmp1, (0, 0))
        #self.Bind(wx.EVT_ERASE_BACKGROUND, self.OnEraseBackground)

            str1 = "%s %dx%d" % (image_file, bmp1.GetWidth(), bmp1.GetHeight())
            parent.SetTitle(str1)
        
        except IOError:
            print "Image file %s not found" % imageFile
            raise SystemExit
        
        self.button1 = wx.Button(
                self.bitmap1, label = 'Button1',
                pos = (8,8))

app = wx.App(False)
