#!/usr/bin/env python2

import sys
import time
from math import ceil, floor

from polarity_find import PolarityFind
#sys.path.append('..')
#from DistanceFind import DistanceFind


import flycapture2 as fc2
import numpy as np
import cv2
import cv

def show_img(img):
    while max(img.shape[:2]) > 500:
        img = cv2.pyrDown(img)
    #ratio = 500./max(img.shape)
    #cv2.resize(img, 

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

    def print_info(self, name, key):
        info = self.get_property_info(key)
        value = self.get_property(key)
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
    def print_infos(self):
        self.print_info('frame rate', fc2.FRAME_RATE)
        self.print_info('auto exposure', fc2.AUTO_EXPOSURE)
        self.print_info('shutter', fc2.SHUTTER)
        self.print_info('gain', fc2.GAIN)



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
c.print_infos()

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
gains = []
frames = []
for gain in iter_values(c, fc2.GAIN, 20, 100):
    c.set_property_manual(fc2.GAIN, gain) # maximum framerate
    f = c.get_frame()
    gains.append(gain)
    frames.append(f)

thresholds = np.linspace(0, 255, 20)
good_gains = []
for i, threshold in enumerate(thresholds):
    finder = PolarityFind(True, 3, threshold=threshold)
    line = [(finder.find_target(frame)[0] is not None) for gain, frame in zip(gains, frames)]
    print 'threshold:', threshold, 'good:', sum(line)
    good_gains.append(line)

good_gains = np.array(good_gains, dtype=np.uint8)
good_gains[good_gains==1] = 255
#print 'good_gains:', good_gains

#while max(good_gains.shape) < 250:
#    good_gains = cv2.pyrUp(good_gains)

ratio = 500./max(good_gains.shape)
good_gains = cv2.resize(good_gains, (0, 0), None, ratio, ratio)
if len(sys.argv) > 1:
    cv2.imwrite(sys.argv[1], good_gains)
    print 'wrote map to', sys.argv[1]
else:
    show_img(good_gains)

'''
# find lowest/highest gain where target is found
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
'''
