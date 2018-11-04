#!/usr/bin/python

###########################################################################
## Python code generated with wxFormBuilder (version Jul  2 2018)
## http://www.wxformbuilder.org/
## WX VERSION 3.0.2
## PYTHON VERSION 2.7
## PLEASE DO *NOT* EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc
import csv
import os

#import rospy
#import rospkg

import start # page that accesses "games"
import demo # page that accesses "games"
###########################################################################
## Class frameMain
###########################################################################

class frameMain ( wx.Frame ):

        def __init__ ( self, parent ):
	    wx.Frame.__init__ ( self, parent, id = wx.ID_ANY, title = u"Main", size = wx.GetDisplaySize(), style = wx.SYSTEM_MENU|wx.MAXIMIZE|wx.TAB_TRAVERSAL )

            #setting up buttons:
            self.submitBtn = wx.Button( self, wx.ID_ANY, label="Submit")
            self.startBtn = wx.Button( self, wx.ID_ANY, label="Start")
            self.demoBtn = wx.Button( self, wx.ID_ANY, label="Demo")
            self.closeBtn = wx.Button(self, wx.ID_ANY, label="X")
            self.minimizeBtn=wx.Button(self, wx.ID_ANY,label="_")
            self.label = wx.StaticText(self, wx.ID_ANY, 'User ID: ')
            self.name = wx.TextCtrl(self, wx.ID_ANY, wx.EmptyString)
            # self.rospack = rospkg.RosPack()

	    # Connect buttons to Events
            self.minimizeBtn.Bind( wx.EVT_BUTTON, self.onMinimize)
	    self.submitBtn.Bind( wx.EVT_BUTTON, self.onName )
	    self.startBtn.Bind( wx.EVT_BUTTON, self.onStart )
	    #self.name.Bind( wx.EVT_TEXT, self.check_string )
            self.demoBtn.Bind( wx.EVT_BUTTON, self.onDemo )
            self.closeBtn.Bind(wx.EVT_BUTTON, self.onClose)
            #set up screen	i

            self.startBtn.Disable()

            self.layout()

      #  def check_string(self, event):
      #      if not (self.name.GetValue()==''):
      #          self.submitBtn.Enable()

        def layout(self):
	    topBox = wx.BoxSizer( wx.HORIZONTAL )#box for buttons close and minimize
	    nameBox = wx.BoxSizer( wx.HORIZONTAL ) #box for name and the text control
            mainBox = wx.BoxSizer( wx.VERTICAL )
            startBox = wx.BoxSizer( wx.HORIZONTAL )#box for buttons start and test

            #setting up boxes of buttons
            topBox.Add( self.minimizeBtn, 0, 0, 0)
            topBox.Add( self.closeBtn, 0, 0 , 0 )

            nameBox.Add( self.label, 2, 0, 0 )
            nameBox.Add( self.name,4,wx.CENTER|wx.EXPAND,0)
            nameBox.Add( self.submitBtn,2, 0, 0)

            startBox.Add( self.demoBtn, 4, 0, 0 )
            startBox.Add( self.startBtn, 4, 0, 0 )

            mainBox.Add( topBox, 4, wx.ALIGN_RIGHT, 0 )
            mainBox.Add( nameBox, 0, wx.ALIGN_CENTER, 0 )
            mainBox.Add( startBox, 4, wx.ALIGN_CENTER, 0 )

            self.SetSizer( mainBox )
	    self.Layout()
	    self.Centre( wx.BOTH )

        def __del__( self ):
	    pass

        def onName( self,event):
            #save on the csv file the name of the user
            # path = self.rospack.get_path('ws_generator')
            # self.csvfile = path + '/src/csvfiles/' + self.name.GetValue() + '.csv'
            self.csvfile = self.name.GetValue() + '.csv'
            self.startBtn.Enable()

        def onMinimize( self, event ):
            self.Hide()

	def onDemo( self, event ):
            f = demo.Frame() #test page opens
	    self.Close()
            f.Show()

	def onStart( self, event ):
            with open(self.csvfile, 'w+') as fout:
                fout.close()

            f = start.Frame(self.csvfile) #start page opens
            self.Close()
            f.Show()

        def onClose( self, event ):
            self.Close()

if __name__ == '__main__':
    app = wx.App()# NECESSARY TO START UYSING WX LIBRARY
    frame = frameMain(None)
    frame.Show()
    app.MainLoop()
