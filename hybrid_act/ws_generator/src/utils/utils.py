#! /usr/bin/env/python3

import numpy as np
import wx

np.set_printoptions(suppress=True)

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


def Generate_WS(obj, ws):
    WS_DIV = 3
    WS_SIZE = int(np.ceil(obj.WIDTH/float(WS_DIV)))

    intensity = np.zeros([len(ws)*2,obj.WIDTH])
    """Determine y workspace bounds"""
    y_ws = np.empty([len(ws),2])
    rectangle_y = obj.FIRST_RECTANGLE_Y
    for i in range(len(ws)):
        y_ws[i] = [rectangle_y,rectangle_y+obj.RECTANGLE_SIZE]
        rectangle_y = rectangle_y + obj.RECTANGLE_SIZE + obj.RECTANGLE_SEPERATION

    HAPTIC_WIDTH = float(obj.HAPTIC_WIDTH)
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
                intensity[2*i][ind[0]+obj.TEXTBOX_WIDTH] = sinusoid[ind[0]]
                intensity[2*i+1][ind[1]+obj.TEXTBOX_WIDTH] = -sinusoid[ind[1]]
            elif actuation == "UFM":
                intensity[2*i] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi) + amp/2
            elif actuation == "EV":
                intensity[2*i+1][obj.TEXTBOX_WIDTH:] = amp/2*np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi) + amp/2

        elif shape  == "Square":
            sinusoid = np.sin(HORIZ_PIXELS/HAPTIC_WIDTH*frequency*2*np.pi)
            ind = [np.where(sinusoid>0)[0], np.where(sinusoid<=0)[0]]

            if actuation == "Hybrid":
                intensity[2*i][ind[0]+obj.TEXTBOX_WIDTH] = amp
                intensity[2*i+1][ind[1]+obj.TEXTBOX_WIDTH] = amp
            elif actuation == "UFM":
                intensity[2*i][ind[0]+obj.TEXTBOX_WIDTH] = amp
            elif actuation == "EV":
                intensity[2*i+1][ind[1]+obj.TEXTBOX_WIDTH] = amp

        elif shape == "Triangular":
            triangle = np.zeros(obj.HAPTIC_WIDTH)
            slope = 2.0*frequency/obj.HAPTIC_WIDTH
            triangle[obj.TEXTBOX_WIDTH] = -1.0
            for j in range(1,len(triangle)):
                if triangle[j-1] >= 1.0:
                    triangle[j] = -1.0
                else:
                    triangle[j] = min(1.0,triangle[j-1]+slope)

            if actuation == "Hybrid":
                ind = [np.where(triangle>0)[0], np.where(triangle<=0)[0]]
                intensity[2*i][ind[1]+obj.TEXTBOX_WIDTH] = -triangle[ind[1]]
                intensity[2*i+1][ind[0]+obj.TEXTBOX_WIDTH] = triangle[ind[0]]
                #print intensity[2*i]
            else:
                positive_triangle = np.array(triangle)
                positive_triangle[:] = triangle[:]/2. + 0.5
                if actuation == "UFM":
                    intensity[2*i][obj.TEXTBOX_WIDTH:] = positive_triangle
                elif actuation == "EV":
                    intensity[2*i+1][obj.TEXTBOX_WIDTH:] = positive_triangle

        elif shape == "Bump":
            x_center = (HAPTIC_WIDTH)/2
            x_biasedcenter = x_center+obj.TEXTBOX_WIDTH

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

            for index in range(int(obj.TEXTBOX_WIDTH),int(obj.WIDTH)):
                if (index <= x_ufm_dropoff[0] or index >= x_ufm_dropoff[1]):
                    intensity[2*i][index] = 1.0
                    intensity[2*i+1][index] = 0.0
                else:
                    ufm_int = aufm*(index-x_biasedcenter)**2+kufm
                    intensity[2*i][index] = max(0,min(1,ufm_int))
                    ev_int = aev*(index-x_biasedcenter)**2+kev
                    intensity[2*i+1][index] = max(0,min(1,ev_int))

    intensity_out = np.zeros([len(intensity),WS_SIZE])
    for j in range(len(intensity)):
        intensity_out[j][:] = intensity[j][::WS_DIV]*100

    return intensity_out.astype(int), y_ws.astype(int)


if __name__ == "__main__":
    obj = test_object(500,500,400,100,100,25,10)
    # ws has form of channel: actuation, amplitude, texture, frequency
    channels = {0: ["Hybrid", 1, "Bump", 10],
                1: ["UFM", 1, "Bump", 10],
                2: ["EV", 1, "Bump", 10]}
    intensity, y_ws = Generate_WS(obj, channels)
    for i in range(len(intensity)):
        print intensity[i]
