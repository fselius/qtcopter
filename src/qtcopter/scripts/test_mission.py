#!/usr/bin/python

from px4flow_test import Flow
from ransac_find import RANSACFind as finder
from ptgrey import PTGreyCamera

# init camera
cam = PTGreyCamera()
# set manual values
c.set_property_manual(fc2.AUTO_EXPOSURE, 0) # exposure = 0, we don't modify this. I'm not sure, but it had no effect.
c.set_property_manual(fc2.SHUTTER, args.shutter) # 10ms shutter (1/100, hopefully fast enough)
# if frame_rate is too high, it is set to maximum :)
c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
c.set_property_manual(fc2.GAIN, args.gain)
c.print_infos()

c.start_capture()

# init flow
#flow = Flow('/dev/ttyACM1')
flow = Flow(dummy=True)
flow.listen()


class Height:
    def __call__(self):
        return 1 # 1 meter
# init height
height = Height()

# init target finder
f = finder()

while True:
    img = c.get_frame()
    center, size = finder.find_target(img)
    if center is not None:
        # found target
        # todo
        # X, Y = ..
        print 'Found target at', center, 'size:', size
        h = height()
        x, y = center[0], center[1]
        x *= 80/800.*h # 80 cm per 800 pixels at 1m? TODO: measure
        y *= 60/600.*h # 60 cm per 600 pixels at 1m?

        flow.set_xy(x, y)
    print 'X=%2.2f Y=%2.2f' % (flow.X, flow.Y)


