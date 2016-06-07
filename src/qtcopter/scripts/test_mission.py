#!/usr/bin/python

from px4flow_test import Flow
import sys
import time
import random
sys.path.append('../src/qtcopter/balldrop')
sys.path.append('../src/qtcopter')
from ransac_find import RANSACFind as finder
from ptgrey import PTGreyCamera
import flycapture2 as fc2

class DummyPTGreyCamera:
    def set_property_manual(self, a, b):
        pass
    def print_infos(self):
        pass
    def start_capture(self):
        pass
    def get_frame(self):
        time.sleep(0.1)
        return None


class DummyFinder:
    def find_target(self, img):
        time.sleep(0.1)
        return (random.random()*800-400, random.random()*600-300), 5
#finder = DummyFinder


shutter = 10
gain = 10
# init camera
c = PTGreyCamera()
# set manual values
c.set_property_manual(fc2.AUTO_EXPOSURE, 0) # exposure = 0, we don't modify this. I'm not sure, but it had no effect.
c.set_property_manual(fc2.SHUTTER, shutter) # 10ms shutter (1/100, hopefully fast enough)
# if frame_rate is too high, it is set to maximum :)
c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
c.set_property_manual(fc2.GAIN, gain)
c.print_infos()
c.start_capture()

# init flow
#flow = Flow('/dev/ttyACM1')
flow = Flow(dummy=True)
flow.run()

class Height:
    def __call__(self):
        return 1.5 # 1.5 meters
# init height
height = Height()

# init target finder
f = finder(True, 3, True)

try:

    while True:
        img = c.get_frame()
        center, size = f.find_target(img)
        if center is not None:
            # found target
            # todo
            # X, Y = ..
            print 'Found target at', center, 'size:', size
            h = height()
            x, y = center[0], center[1]
            x *= 0.80/800.*h # 80 cm per 800 pixels at 1m? TODO: measure
            y *= 0.60/600.*h # 60 cm per 600 pixels at 1m?

            flow.set_xy(x, y)
        print 'X=%2.2f Y=%2.2f' % (flow.X, flow.Y)
except:
    flow.stop()
    raise


