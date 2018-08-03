
import wx
import math
import random


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

class BallPanel(wx.Window):
    def __init__(self, parent, refresh, length):
        wx.Window.__init__(self, parent)
        self.ball = []
        self.last_pos = self.ScreenToClient(wx.GetMousePosition())
        self.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.SetBackgroundColour("WHITE")

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, self.on_size)
        self.WIDTH,self.HEIGHT = self.GetClientSize()
        self.BALL_RADIUS = 40
        self.BALL_VELOCITY = 10       #cm/s
        self.REFRESH = refresh
        self.LENGTH = length
        self.ball_start = [[self.BALL_RADIUS+10,self.BALL_RADIUS], [self.BALL_RADIUS+10,self.BALL_RADIUS*4]]
        self.WAIT = 10

        self.wait_count = []
        for l in self.ball_start:
            self.ball.append(Ball(l,self.BALL_RADIUS,self.WIDTH-self.BALL_RADIUS*1.5))
            self.wait_count.append(0)

        wx.CallLater(200, self.SetFocus)
        self.on_size(0)
        self.update_drawing()


    def on_size(self, event):
        self.WIDTH, self.HEIGHT = self.GetClientSize()
        # self._buffer = wx.Bitmap(self.WIDTH, self.HEIGHT)
        for ball in self.ball:
            ball.update_limit(self.WIDTH-2*self.BALL_RADIUS)
        self.BALL_MOVEX = int(self.BALL_VELOCITY*self.REFRESH*self.WIDTH/(2450.*self.LENGTH))
        self.update_drawing()

    def update_drawing(self):
        self.Refresh(True)

    def on_paint(self, event):
        x, y = self.ScreenToClient(wx.GetMousePosition())
        dc = wx.AutoBufferedPaintDC(self)

        for i,ball in enumerate(self.ball):
            if ball.x - self.BALL_RADIUS <= x <= ball.x + self.BALL_RADIUS:
                if ball.y - self.BALL_RADIUS <= y <= ball.y + self.BALL_RADIUS:
                    self.wait_count[i] = 0
                    dc.Clear()
                    ball.move_forward(dc,self.BALL_MOVEX)
                    break
            elif ball.x != self.ball_start[i][0]:
                if self.wait_count[i] < self.WAIT:
                    self.wait_count[i] += 1
                    break
                else:
                    dc.Clear()
                    self.wait_count[i] = 0
                    ball.move_ball(dc,self.ball_start[i])
            else:
                ball.move_ball(dc,self.ball_start[i])

class BallFrame(wx.Frame):
    def __init__(self, *args, **kw):
        wx.Frame.__init__(self, *args, **kw)

        self.Bind(wx.EVT_CLOSE, self.on_close)
        self.Bind(wx.EVT_TIMER, self.on_timer)

        self.REFRESH_RATE = 20
        self.SCREEN_LENGTH = 21.5625                    #in
        self.panel = BallPanel(self, self.REFRESH_RATE, self.SCREEN_LENGTH)
        self.timer = wx.Timer(self)
        self.timer.Start(self.REFRESH_RATE)

    def on_close(self, event):
        self.timer.Stop()
        self.Destroy()

    def on_timer(self, event):
        self.panel.update_drawing()


app = wx.App(False)
frame = BallFrame(None, -1, "Balls!")
frame.Show(True)
app.MainLoop()


# In[12]:


del app

