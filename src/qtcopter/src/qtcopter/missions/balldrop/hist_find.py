#!/usr/bin/env python
'''
Use histogram to locate target

for IMG_0527.JPG, target is at 330, 220 -> 390, 280
center at 360, 250

'''

import sys
import os
import argparse
import time

import cv2
import pylab
import numpy as np
from math import ceil


# constants
INPUT_WIDTH = 500
RATIO_INPUT_TO_RECT = 1.0/20
RATIO_RECT_TO_OVERLAP = 1.0/2

RECT_SIZE_X = int(ceil(INPUT_WIDTH*RATIO_INPUT_TO_RECT))
RECT_SIZE_Y = int(ceil(INPUT_WIDTH*RATIO_INPUT_TO_RECT))
RECT_OVERLAP_X = int(ceil(RECT_SIZE_X*RATIO_RECT_TO_OVERLAP))
RECT_OVERLAP_Y = int(ceil(RECT_SIZE_Y*RATIO_RECT_TO_OVERLAP))
if False:
    INPUT_WIDTH = 500
    RECT_SIZE_X = 40
    RECT_SIZE_Y = 40
    RECT_OVERLAP_X = 20
    RECT_OVERLAP_Y = 20 
if False:
    # smaller picture
    INPUT_WIDTH = 100
    RECT_SIZE_X = 10
    RECT_SIZE_Y = 10
    RECT_OVERLAP_X = 4
    RECT_OVERLAP_Y = 4 



########
# dummy @profile decorator
import __builtin__

try:
    __builtin__.profile
except AttributeError:
    # No line profiler, provide a pass-through version
    def profile(func): return func
    __builtin__.profile = profile

#########

@profile
def find_edges(hist, cut=0.05):
    """ find edges of histogram (for the black-white case). Usually, the
    colors will be either over exposed (white is white, black is gray),
    or under exposed (white is gray, black is black).
    We find actual edges, rather than doing a histogram equalization, which
    isn't exactly what we want, and it takes away the beautiful peaks that
    we would like to find.
    
    hist - histogram
    cut - how much we want to cut of the histogram (cut from each side)
    """
    
    total = hist.sum()
    cut = np.float32(cut*float(total))
    #cut = cut*total # slower..

    # find low edge
    sums = hist.cumsum()
    i = np.argmax(sums>cut)
    low = max(0, i-1) # if i < 0..

    # find high edge
    i = np.argmax(sums>(total-cut))
    high = min(len(hist)-1, i+1) # if i >= len(hist)

    return low, high

@profile
def is_black_white(hist):
    " decide whether histogram is of black/white pattern "
    low, high = find_edges(hist)    
    #print 'low, high, total hist length:', low, high, len(hist)

    #hist = hist[low:high+1]

    # take lower and upper % buckets (darkest + lightest), if they
    # contain more than threshold, good
    BUCKETS = 0.05 # precentage of lower and upper
    THRESHOLD = 0.5 # how much these buckets should have

    buckets = int(BUCKETS*(high-low+1))
    #buckets = int(BUCKETS*len(hist))
    # TODO: in case BUCKETS*len(hist) is not a whole, should we take part
    # of the next bucket as well?
    sides_sum = hist[low:low+buckets].sum() + hist[high-buckets:high+1].sum()
    #sides_sum = sum(hist[:buckets]) + sum(hist[-buckets:])
    #print 'low:', 1.0*sum(hist[:buckets])/sum(hist), 'high:', 1.0*sum(hist[-buckets:])/sum(hist), 'total:', 1.0*sides_sum/sum(hist)
    #if 1.0*sides_sum/sum(hist) > THRESHOLD:
    return 1.0*sides_sum/hist[low:high+1].sum() > THRESHOLD
    #return 1.0*sides_sum > hist[low:high+1].sum()*THRESHOLD

@profile
def hist_rect(channel, x=0, y=0, width=None, height=None, max_value=255):
    " calculate histogram in rectangle "
    ranges = [0, max_value+1]
    dims = [max_value+1]

    # defaults
    if width is None:
        width = channel.shape[1]-x
    if height is None:
        height = channel.shape[0]-y
    
    # create mask of pixels we want to calc histogram of
    mask = np.zeros(channel.shape, dtype=np.uint8)
    mask[y:y+height, x:x+width] = np.ones((width, height))
    # calc histogram :)
    hist = cv2.calcHist([channel], [0], mask, dims, ranges)
    return hist.reshape(hist.size) # return a flat array

def iter_rect(x, y, width, height, rect_width, rect_height, overlap_x, overlap_y):
    " generate rectangle coordinates "
    i=0
    # TODO: Currently we don't handle cases when right\bottom edge isn't
    # at round boundary (i.e. not multiple of rect_width-overlap_x)
    # we should probably also yield last row/column end the end to check
    # these edges.
    for row in range(x, width-rect_width, rect_width-overlap_x):
        for col in range(y, height-rect_height, rect_height-overlap_y):
            yield row, col, rect_width, rect_height
            i+=1
    #print 'loops:', i

@profile
def hist_iter_rects(channel,rect_width, rect_height, overlap_x, overlap_y, hist_filter=is_black_white):
    " iterate over good rectangles "
    # TODO: perhaps rewrite with filter()
    it_rects = iter_rect(0, 0, channel.shape[1], channel.shape[0],
            rect_width, rect_height, overlap_x, overlap_y)
    for x, y, width, height in it_rects:
        hist = hist_rect(channel, x, y, width, height, max_value=255)
        if hist_filter(hist):
            yield x, y

def roi_hist(channel, hist_filter=is_black_white,
            resize=500, # None to not resize
            ratio_size_to_rect=RATIO_INPUT_TO_RECT,
            ratio_rect_to_overlap=RATIO_RECT_TO_OVERLAP):
    " return ROI where hist_filter() returns true "
    original_shape = channel.shape

    if resize is not None:
        # resize
        ratio = 1.0*resize/max(channel.shape[:2])
        channel = cv2.resize(channel, (0,0), fx=ratio, fy=ratio)

    # make rectangle size
    height, width = channel.shape # TODO: make sure this is the order
    rect_size_x = int(ceil(width*ratio_size_to_rect))
    rect_size_y = int(ceil(height*ratio_size_to_rect))
    rect_overlap_x = int(ceil(rect_size_x*ratio_rect_to_overlap))
    rect_overlap_y = int(ceil(rect_size_y*ratio_rect_to_overlap))

    # calculate small roi
    roi = roi_hist_ex(channel, hist_filter,
                    rect_size_x, rect_size_y, rect_overlap_x, rect_overlap_y)
    
    if resize is not None:
        # resize roi
        roi = cv2.resize(roi, original_shape)
    return roi

def roi_hist_ex(channel, hist_filter, rect_size_x, rect_size_y, rect_overlap_x, rect_overlap_y):
    " return ROI where hist_filter() returns true "
    roi = np.zeros(channel.shape) #, dtype=np.bool)

    good_rects = hist_iter_rects(channel,
            rect_size_x, rect_size_y, rect_overlap_x, rect_overlap_y,
            hist_filter=hist_filter)
    for x, y in good_rects:
        pts = [[x, y], [x+rect_size_x, y], [x+rect_size_x, y+rect_size_y], [x, y+rect_size_y]]
        #pts = [[y, x], [y, x+rect_width], [y+rect_height, x+rect_width], [y+rect_width, x]]

        roi[y:y+rect_size_y, x:x+rect_size_x] = np.ones((rect_size_y, rect_size_x))

    return roi
    
def main():
    parser = argparse.ArgumentParser(description='Find balldrop target using histogram')
    parser.add_argument('images', nargs='+', help='input images')
    parser.add_argument('--view', action='store_true', help='view results')
    #parser.add_argument('--save', '-s', action='store_true', help='save results to IMG.target.jpg')
    parser.add_argument('--channel', '-c', default=1, type=int, help='channel (HLS) number to use')
    parser.add_argument('--verbose', '-v', action='count')
    parser.add_argument('--benchmark', action='store_true', help='run some benchmarks instead')

    args = parser.parse_args()
    for img_path in args.images: 
        # open image
        img = cv2.imread(img_path)
        if img is None:
            print "Could not open file %s" % (img_path,)
            continue

        t = time.time()

        # resize to have maximum 500px width/height
        ratio = 1.0*INPUT_WIDTH/max(img.shape[:2])
        img = cv2.resize(img, (0,0), fx=ratio, fy=ratio)

        img_orig = img

        # convert to HLS
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

        # take only the relevant channel (we probably want light)
        channels = cv2.split(img)
        img_channel = channels[args.channel]

        # for Hue the ranges are 0-179. Saturate & Light are 0-255.
        if args.channel == 0:
                max_value = 179
        else:
                max_value = 255

        # just run a benchmark if requested
        if args.benchmark:
            print 'Benchmark on %s' % (img_path,)
            N = 1000
            t = time.time()
            for i in range(N):
                    #hist = hist_rect(img_channel, 220, 330, 40, 40, max_value=max_value)
                    hist = hist_rect(img_channel, 0, 0, None, None, max_value=max_value)
            print 'hist:', (time.time()-t)/N

            N = 1000
            t = time.time()
            for i in range(N):
                    x = is_black_white(hist)
            print 'is_black_white:', (time.time()-t)/N
            continue
            
        # calc hist
        rect_width = RECT_SIZE_X
        rect_height = RECT_SIZE_Y
        overlap_x = RECT_OVERLAP_X
        overlap_y = RECT_OVERLAP_Y

        roi = roi_hist(img_channel)
        img_orig[roi==True] = (0, 0, 255)
        '''
        for x, y in hist_iter_rects(img_channel, rect_width, rect_height, overlap_x, overlap_y):
            # draw a rectangle around found qrcodes
            if args.verbose > 1:
                print x, y
            pts = [[x, y], [x+rect_width, y], [x+rect_width, y+rect_height], [x, y+rect_width]]
            #pts = [[y, x], [y, x+rect_width], [y+rect_height, x+rect_width], [y+rect_width, x]]

            poly1 = np.array(pts, np.int32) #.reshape((-1,1,2))
            polys = [poly1]
            cv2.polylines(img_orig, polys, True, (0, 0, 255))
        '''

        if args.verbose > 0:
            print '%fs for %s' % (time.time()-t, img_path)
        if args.view:
            cv2.imshow('bah', img_orig)
            while True:
                if cv2.waitKey(1)&0xff == ord('q'):
                    break
                
            cv2.destroyWindow('bah')

if __name__=='__main__':
    main()

