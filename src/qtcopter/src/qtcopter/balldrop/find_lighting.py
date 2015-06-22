#!/usr/bin/env python2

#from qtcopter.balldrop.polarity_find import PolarityFind
#from qtcopter.missions.DetailedFind import DetailedFind


import sys
import time
from math import ceil, floor
sys.path.append('..')
from DistanceFind import DistanceFind

from polarity_find import PolarityFind

import flycapture2 as fc2
import numpy as np
import cv2
import cv

def show_img(img):
    #ratio = 500./max(img.shape)
    #cv2.resize(img, 
    img = cv2.pyrDown(img)

    cv2.imshow('model', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


videomodes = filter(lambda _: _.startswith('VIDEOMODE_'), dir(fc2))
VIDEOMODE = dict((fc2.__getattribute__(k), k) for k in videomodes)
framerates = filter(lambda _: _.startswith('FRAMERATE_'), dir(fc2))
FRAMERATE = dict((fc2.__getattribute__(k), k) for k in framerates)
pixelformats = filter(lambda _: _.startswith('PIXEL_FORMAT_'), dir(fc2))
PIXEL_FORMAT = dict((fc2.__getattribute__(k), k) for k in pixelformats)

class PTGreyCamera(object):
    def __init__(self):
        self.capturing = False
        self.context = fc2.Context()
        print 'number of cameras:', self.context.get_num_of_cameras()
        #=c.set_video_mode_and_frame_rate(fc2.VIDEOMODE_1280x960Y16, fc2.FRAMERATE_7_5)
        if self.context.get_num_of_cameras() == 0:
            raise RuntimeError('No camera connected')
        self.context.connect(*self.context.get_camera_from_index(0))
        print 'camera info:', self.context.get_camera_info()

    def get_frame(self):
        im = fc2.Image()
        self.context.retrieve_buffer(im)
        #print 'get_frame im:', dir(im), 'size:', im.__sizeof__(), 'format:', PIXEL_FORMAT[im.get_format()]
        #print 'rows, cols, stride', im.get_rows(), im.get_cols(), im.get_stride()
        #print 'bayerformat, datasize, received datasize', im.get_bayerFormat(), im.get_dataSize(), im.get_receivedDataSize()
        a = np.array(im)
        # convert Bayer -> RGB
        # These two work: GB RGB, GR BGR (fc2BayerTileFormat = 3 = FC2_BT_GBRG, /**< Green-Blue-Red-Green. */)
        return cv2.cvtColor(a, cv.CV_BayerGR2BGR)
    def get_frame_bw(self):
        f = self.get_frame()
        return cv2.cvtColor(f, cv.CV_RGB2GRAY)

    def start_capture(self):
        self.capturing = True
        self.context.start_capture()
    def stop_capture(self):
        if self.capturing:
            self.context.stop_capture()
            self.capturing = False
    def __del__(self):
        self.stop_capture()
        self.context.disconnect()

    def set_property_auto(self, prop):
        p = self.context.get_property(prop)
        p['auto_manual_mode'] = True
        return self.context.set_property(**p)
    def set_property_manual(self, prop, value):
        p = self.context.get_property(prop)
        p['abs_value'] = value
        p['abs_control'] = True
        p['auto_manual_mode'] = False
        return self.context.set_property(**p)
    def get_property(self, prop):
        return self.context.get_property(prop)
    def get_property_info(self, prop):
        return self.context.get_property_info(prop)

def print_info(context, name, key):
    info = context.get_property_info(key)
    value = context.get_property(key)
    print_data = {
        'name': name,
        'min': info['abs_min'],
        'max': info['abs_max'],
        'units': info['units'],
        'cur': value['abs_value'],
        'auto': value['auto_manual_mode'],
        'abs_control': value['abs_control']
    }
    print '{name}: min={min:.3f} max={max:.3f} cur={cur:.3f} [{units}] auto={auto} abs_control={abs_control}'.format(**print_data)
def print_infos(c):
    print_info(c, 'frame rate', fc2.FRAME_RATE)
    print_info(c, 'auto exposure', fc2.AUTO_EXPOSURE)
    print_info(c, 'shutter', fc2.SHUTTER)
    print_info(c, 'gain', fc2.GAIN)



c = PTGreyCamera()

# video mode
m, f = c.context.get_video_mode_and_frame_rate()
print 'video mode', m, f, VIDEOMODE[m], FRAMERATE[f]
print 'video mode info:', c.context.get_video_mode_and_frame_rate_info(m, f)

# set video mode
#x = c.context.set_video_mode_and_frame_rate(fc2.VIDEOMODE_1280x960RGB, fc2.FRAMERATE_3_75)
#print 'set mode:', x
#m, f = c.context.get_video_mode_and_frame_rate()
#print 'video mode', m, f, VIDEOMODE[m], FRAMERATE[f]
#print 'video mode info:', c.context.get_video_mode_and_frame_rate_info(m, f)

# set manual values
c.set_property_manual(fc2.AUTO_EXPOSURE, 0) # exposure = 0, we don't modify this. I'm not sure, but it had no effect.
c.set_property_manual(fc2.SHUTTER, 10) # 10ms shutter (1/100, hopefully fast enough)
# if frame_rate is too high, it is set to maximum :)
c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
c.set_property_manual(fc2.GAIN, 100) # maximum gain
print_infos(c)

c.start_capture()

def iter_values(context, prop, count, try_max=None):
    info = c.get_property_info(prop)
    start, stop = info['abs_min'], info['abs_max']
    if try_max is not None:
        # apparently sometimes a higher than maximum value is possible, so we
        # simply try that
        c.set_property_manual(prop, try_max)
        x = c.get_property(prop)
        if x['abs_value'] > stop:
            print 'iter_values: using {0} instead {1}'.format(x['abs_value'], stop)
            stop = x['abs_value']
    return np.linspace(start, stop, count)

# get frames using different gains
frames = []
for gain in iter_values(c, fc2.GAIN, 20, 100):
    c.set_property_manual(fc2.GAIN, gain) # maximum framerate
    f = c.get_frame()
    frames.append((gain, f))

finder = PolarityFind(True, 3)
# find lowest/highest gain where target is found
# TODO: Well, this is a lie: finder is using a parameter we want to calibrate.
# We need to take that into consideration as well.
gains_i = [i for i, (gain, f) in enumerate(frames) if finder.find_target(f)[0] is not None]
for i, (gain, f) in enumerate(frames):
    if i in gains_i:
        print gain, ':)'
    else:
        print gain

if not gains_i:
    print 'Could not find target :-('
    sys.exit()

high = max(gains_i)
# TODO: I noticed that lowest gain is actually slightly higher than the next
# one.. check if it's true
low = min(gains_i)
# if they are not boundary values, include the next one
if high < len(frames)-1:
    high += 1
if low > 0:
    low -= 1

low, high = frames[low][0], frames[high][0]
print 'low, high gains:', low, high


