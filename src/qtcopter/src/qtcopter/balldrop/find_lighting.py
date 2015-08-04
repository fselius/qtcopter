#!/usr/bin/env python2

import sys
import time
from math import ceil, floor
import argparse

from polarity_find import PolarityFind
#sys.path.append('..')
#from DistanceFind import DistanceFind


import flycapture2 as fc2
import numpy as np
import cv2
import cv

from qtcopter.ptgrey import PTGreyCamera, VIDEOMODE, FRAMERATE

def show_img(img):
    while max(img.shape[:2]) > 500:
        img = cv2.pyrDown(img)
    #ratio = 500./max(img.shape)
    #cv2.resize(img, 

    cv2.imshow('model', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Find gain and threshold for balldrop.')
    parser.add_argument('--shutter', default=10, type=int, help='shutter time (10 ms default for indoor, 1-2 ms is fine for outside)')
    parser.add_argument('--output', '-o', help='save map to file')
    parser.add_argument('--show', action='store_true', help='show result map')
    args = parser.parse_args()


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
    c.set_property_manual(fc2.SHUTTER, args.shutter) # 10ms shutter (1/100, hopefully fast enough)
    # if frame_rate is too high, it is set to maximum :)
    c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
    c.set_property_manual(fc2.GAIN, 100) # maximum gain
    c.print_infos()

    c.start_capture()

    def iter_values(context, prop, count, try_harder=True):
        low, high = context.get_property_range(prop, try_harder=try_harder)
        return np.linspace(low, high, count)

    # get frames using different gains
    gains = []
    frames = []
    for gain in iter_values(c, fc2.GAIN, 20, 100):
        c.set_property_manual(fc2.GAIN, gain)
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
    for i, gain in enumerate(gains):
        print 'gain:', gain, 'good:', sum(good_gains[:,i])/255

    ratio = 500./max(good_gains.shape)
    good_gains = cv2.resize(good_gains, (0, 0), None, ratio, ratio)
    if args.output:
        cv2.imwrite(args.output, good_gains)
        print 'wrote map to', args.output
    show_img(good_gains)

