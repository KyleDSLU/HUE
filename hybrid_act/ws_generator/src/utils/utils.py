#! /usr/bin/env/python3

import numpy as np
import wx

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

class test_object():
    def __init__(self, width, height, hw, tbw, fry, rsize, rsep):
        self.WIDTH = width
        self.HEIGHT = height
        self.HAPTIC_WIDTH = hw
        self.TEXTBOX_WIDTH = tbw
        self.FIRST_RECTANGLE_Y = fry
        self.RECTANGLE_SIZE = rsize
        self.RECTANGLE_SEPERATION = rsep

class ws_generator():
    def __init__(self, obj):
        self.obj = obj
        self.WS_SIZE = int(obj.WIDTH/2.)
        self.WS_DIV = int(self.obj.WIDTH/self.WS_SIZE)

    def generate_ws(self,ws):
        intensity = np.zeros([len(ws)*2,self.obj.WIDTH])
        """Determine y workspace bounds"""
        y_ws = np.empty([len(ws),2])
        rectangle_y = self.obj.FIRST_RECTANGLE_Y
        for i in range(len(ws)):
            y_ws[i] = [rectangle_y,rectangle_y+self.obj.RECTANGLE_SIZE]
            rectangle_y = rectangle_y + self.obj.RECTANGLE_SIZE + self.obj.RECTANGLE_SEPERATION

        HAPTIC_WIDTH = float(self.obj.HAPTIC_WIDTH)
        HORIZ_PIXELS = np.arange(HAPTIC_WIDTH)

        # ws has form of channel: actuation, amplitude, texture, frequency
        for i,value in enumerate(ws.values()):
            actuation = value[0]
            amp = int(value[1]*100)
            shape = value[2]
            frequency = value[3]

            if shape == "Sinusoid":
                if actuation == "Hybrid":
                    sinusoid = amp*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi)
                    ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]
                    intensity[2*i][ind[0]+self.obj.TEXTBOX_WIDTH] = sinusoid[ind[0]]
                    intensity[2*i+1][ind[1]+self.obj.TEXTBOX_WIDTH] = -sinusoid[ind[1]]
                elif actuation == "UFM":
                    intensity[2*i] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi) + amp/2
                elif actuation == "EV":
                    intensity[2*i+1][self.obj.TEXTBOX_WIDTH:] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi) + amp/2

            elif shape  == "Square":
                sinusoid = np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi)
                ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]

                if actuation == "Hybrid":
                    intensity[2*i][ind[0]+self.obj.TEXTBOX_WIDTH] = amp
                    intensity[2*i+1][ind[1]+self.obj.TEXTBOX_WIDTH] = amp
                elif actuation == "UFM":
                    intensity[2*i][ind[0]+self.obj.TEXTBOX_WIDTH] = amp
                elif actuation == "EV":
                    intensity[2*i+1][ind[1]+self.obj.TEXTBOX_WIDTH] = amp

            elif shape == "Triangular":
                triangle = np.zeros(len(self.obj.WIDTH))
                slope = 2.0*frequency/self.obj.HAPTIC_WIDTH
                triangle[self.TEXTBOX_WIDTH] = -1.0
                for i in range(1,len(triangle)):
                    if triangle[i-1] >= 1.0:
                        triangle[i] = -1.0
                    else:
                        triangle[i] = min(1.0,triangle[i]+slope)
                if actuation == "Hybrid":
                    ind = [np.where(triangle>0)[0], np.where(triangle<=0)[0]]
                    intensity[2*i][ind[1]] = triangle[ind[1]]
                    intensity[2*i+1][ind[0]] = triangle[ind[0]]
                else:
                    positive_triangle = np.array(triangle)
                    positive_triangle[self.obj.TEXTBOX_WIDTH] = triangle[self.obj.TEXTBOX_WIDTH:]/2. + 0.5
                    if actuation == "UFM":
                        intensity[2*i][0] = positive_triangle
                    elif actuation == "EV":
                        intensity[2*i][1] = positive_triangle

            elif shape == "Bump":
                x_center = (HAPTIC_WIDTH)/2
                x_biasedcenter = x_center+self.obj.TEXTBOX_WIDTH

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

                for index in range(int(self.obj.TEXTBOX_WIDTH),int(self.obj.WIDTH)):
                    if (index <= x_ufm_dropoff[0] or index >= x_ufm_dropoff[1]):
                        intensity[2*i][index] = 1.0
                        intensity[2*i+1][index] = 0.0

                    else:
                        ufm_int = aufm*(index-x_biasedcenter)**2+kufm
                        intensity[2*i][index] = max(0,min(1,ufm_int))

                        ev_int = aev*(index-x_biasedcenter)**2+kev
                        intensity[2*i+1][index] = max(0,min(1,ev_int))

        intensity_out = np.zeros([len(intensity),self.WS_SIZE])
        for i in range(len(intensity)):
            intensity_out[i] = intensity[i][::self.WS_DIV]

        return intensity_out.astype(int), y_ws.astype(int)


if __name__ == "__main__":
    obj = test_object(1000,1000,750,250,100,25,10)
    ws_gen = ws_generator(obj)
    # ws has form of channel: actuation, amplitude, texture, frequency
    channels = {0: ["Hybrid", 1, "Sinusoid", 10],
                1: ["EV", 1, "Sinusoid", 10]}
    intensity, y_ws = ws_gen.generate_ws(channels)
    print(intensity)
    print(y_ws)
