#!/usr/bin/python/env python

import smbus
import time

class MAX518_Controller(object):

    def __init__(self, address):
        self._i2cbus = smbus.SMBus(1)
        self._address = address

    def DAC_output(self, output1, output2):
        output1 = int(output1/5.0*255)
        output2 = int(output2/5.0*255)

        #self._i2cbus.write_byte_data(self._address, 0x00, self._address)
        self._i2cbus.write_byte_data(self._address, 0x00, output1)
        self._i2cbus.write_byte_data(self._address, 0x01, output2)

    def MAX518_close(self):
        self.DAC_output(0,0)
        self._i2cbus.close()
        self._i2cbus = None
        
