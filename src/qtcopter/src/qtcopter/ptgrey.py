#!/usr/bin/env python2

import sys
from time import time
from math import ceil, floor
import argparse

import flycapture2 as fc2
import numpy as np
import cv2
import cv


def show_img(img, wait=True, title='bah'):
    while max(img.shape[:2]) > 500:
        img = cv2.pyrDown(img)

    cv2.imshow(title, img)
    if wait:
        while True:
            if cv2.waitKey(0)&0xff==ord('q'):
                break
        cv2.destroyAllWindows()
    else:
        if cv2.waitKey(1)&0xff == ord('q'):
            raise KeyboardInterrupt()

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

    # raw set/get
    def set_property(self, params):
        return self.context.set_property(**params)
    def get_property(self, prop):
        return self.context.get_property(prop)
    def get_property_info(self, prop):
        return self.context.get_property_info(prop)

    # convenient set/get
    def set_property_auto(self, prop):
        p = self.get_property(prop)
        p['auto_manual_mode'] = True
        return self.set_property(p)
    def set_property_manual(self, prop, value):
        p = self.get_property(prop)
        p['abs_value'] = value
        p['abs_control'] = True
        p['auto_manual_mode'] = False
        return self.set_property(p)
    def get_property_range(self, prop, try_harder=False):
        info = self.get_property_info(prop)
        low, high = info['abs_min'], info['abs_max']
        if try_harder:
            # try higher max / lower min values
            # what I noticed with gain property is that it's set like this:
            # abs_value = min + (req-min)%(max-min)
            # min, max = actual minimum/maximum abs_value, req = requested abs_value
            # while abs_min/abs_max returned by get_property_info are not
            # necessarily the actual max/min, but perhaps a bit narrower limits.
            orig = self.get_property(prop)

            # try a higher than maximum value
            jump = (high-low)/100.
            for try_high in np.arange(high+jump, high+jump*100, jump):
                # try a higher value as long as we succeed
                self.set_property_manual(prop, try_high)
                # if value is lower than abs_high, we passed the real max value
                if self.get_property(prop)['abs_value'] < high:
                    break
            new_high = try_high-jump
            if new_high > high:
                print 'Using higher than maximum value! (%r > %r)' % (new_high, high)
                high = new_high

            # try a lower than minimum value
            jump = (high-low)/100.
            for try_low in np.arange(low-jump, low-jump*100, -jump):
                self.set_property_manual(prop, try_low)
                if self.get_property(prop)['abs_value'] > low:
                    break
            new_low = try_low+jump
            if new_low < low:
                print 'Using lower than minimum value! (%r -> %r < %r)' % (try_low, new_low, low)
                low = new_low
            # revert to original value
            self.set_property(orig)
        return low, high

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

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Find gain and threshold for balldrop.')
    parser.add_argument('--shutter', default=10, type=float, help='shutter time [ms] (10 ms default for indoor, 1-2 ms is fine for outside)')
    parser.add_argument('--gain', default=0, type=float, help='gain (-5-20~, default 0)')
    args = parser.parse_args()

    c = PTGreyCamera()

    # video mode
    m, f = c.context.get_video_mode_and_frame_rate()
    print 'video mode', m, f, VIDEOMODE[m], FRAMERATE[f]
    #print 'video mode info:', c.context.get_video_mode_and_frame_rate_info(m, f)

    # set video mode
    #x = c.context.set_video_mode_and_frame_rate(fc2.VIDEOMODE_1280x960RGB, fc2.FRAMERATE_3_75)
    #print 'set mode:', x
    #m, f = c.context.get_video_mode_and_frame_rate()
    #print 'video mode', m, f, VIDEOMODE[m], FRAMERATE[f]
    #print 'video mode info:', c.context.get_video_mode_and_frame_rate_info(m, f)

    # set manual values
    c.set_property_manual(fc2.AUTO_EXPOSURE, 0) # exposure = 0, we don't modify this. I'm not sure, but it had no effect.
    c.set_property_manual(fc2.SHUTTER, args.shutter) # 10ms shutter (1/100, hopefully fast enough)
    # if frame_rate is too high, it is set to maximum :)
    c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
    c.set_property_manual(fc2.GAIN, args.gain) # maximum gain
    c.print_infos()

    c.start_capture()
    while True:
        t = time()
        f = c.get_frame()
        t = time()-t
        print '%.3fs, %.2f fps' % (t, 1/t)
        try:
            pass
            show_img(f, wait=False)
        except KeyboardInterrupt:
            break

    c.stop_capture()


    
