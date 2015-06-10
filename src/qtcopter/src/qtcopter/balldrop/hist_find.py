#!/usr/bin/env python
'''
Use histogram to locate target

for IMG_0527.JPG, target is at 330, 220 -> 390, 280
center at 360, 250

'''

import cv2
import sys
import numpy as np
from math import ceil
from .utils import filter_contours
import rospy

from qtcopter.navigation.Camera import Camera


# constants
INPUT_WIDTH = 500
RATIO_INPUT_TO_RECT = 1.0/20
RATIO_RECT_TO_OVERLAP = 1.0/2
HLS_LIGHT_CHANNEL = 1

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


def debug_image(img, max_size=500):
    if max(img.shape[:2]) > max_size:
        ratio = 1.0*max_size/max(img.shape[:2])
        img = cv2.resize(img, (0, 0), fx=ratio, fy=ratio)
    cv2.imshow('bah', img)
    while True:
        if cv2.waitKey(1)&0xff == ord('q'):
            break

    cv2.destroyWindow('bah')

class HistogramFind(object):
    def __init__(self, diameter, resize=500, channel=HLS_LIGHT_CHANNEL):
        self.resize = resize
        self.channel = channel
        self._radius = diameter/2.0

    def find_roi(self, image, height, camera):
        " Find ROI enclosing rectangle "
        # resize
        ratio = 1.0*self.resize/max(image.shape[:2])
        if ratio < 1:
            image = cv2.resize(image, (0,0), fx=ratio, fy=ratio)

        # find size of rectangle
        rect_size = map(lambda _: int(ceil(_*ratio)), self.get_rect_size(self._radius, height, camera))
        rect_overlap = map(lambda _: int(ceil(RATIO_RECT_TO_OVERLAP*_)), rect_size)
        roi = find_roi(image, channel=self.channel,
                rect_size=rect_size, overlap=rect_overlap)
        if roi is None:
            return None
        # resize back
        if ratio < 1:
            roi = (roi[0][0]/ratio, roi[0][1]/ratio), (roi[1][0]/ratio, roi[1][1]/ratio)
        return roi

    def find_roi_mask(self, image, height, camera):
        " Find ROI mask "
        # resize
        ratio = 1.0*self.resize/max(image.shape[:2])
        if ratio < 1:
            original_shape = image.shape
            image = cv2.resize(image, (0,0), fx=ratio, fy=ratio)
        image = get_light(image, channel=self.channel)
        # find size of rectangle
        rect_size = map(lambda _: int(ceil(_*ratio)), self.get_rect_size(self._radius, height, camera))
        rospy.logdebug('HistogramFind rect_size=%r, image size=%r' % (rect_size, image.shape))
        rect_overlap = map(lambda _: int(ceil(RATIO_RECT_TO_OVERLAP*_)), rect_size)
        roi = roi_hist_ex(image, rect_size, rect_overlap)
        # resize back
        if ratio < 1:
            roi = cv2.resize(roi, original_shape[:2][::-1])
        return roi

    def find_roi_contours(self, image, height, camera):
        " Find ROI contours "
        roi = self.find_roi_mask(image, height, camera)
        contours, hier = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # filter out bad contours - TODO: make sure this works
        #contours = filter_contours(contours, height, camera)
        # sort by area
        return sorted(contours, reverse=True, key=lambda c: cv2.contourArea(c))

    def find_roi_rects(self, image, height, camera):
        contours = self.find_roi_contours(image, height, camera)

        return [cv2.boundingRect(c) for c in contours]

    @staticmethod
    def get_rect_size(radius, height, camera):
        " Get rectangle size for histogram "
        zero = camera.get_camera_offset((0, 0), height) # yep, this will most likely be zero..
        # TODO: Decide how big the histogram should be. Currently as big as target radius.
        bah = camera.get_camera_offset((radius, radius), height)
        rect_size = abs(bah[0]-zero[0]), abs(bah[1]-zero[1])
        print(rect_size)
        return rect_size

def get_light(image, channel=HLS_LIGHT_CHANNEL):
    " Get light channel of image "
    assert(len(image.shape) == 3)
    # convert to HLS
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    # take relevant channel
    x = cv2.split(image)[channel]
    #x = cv2.blur(x, (3, 3))
    return x

def resize(img, max_size):
    " resize image to have maximum width/height "
    ratio = 1.0*max_size/max(img.shape[:2])
    if ratio >= 1:
        return img
    img = cv2.resize(img, (0,0), fx=ratio, fy=ratio)
    return img

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

    # take lower and upper % buckets (darkest + lightest), if they
    # contain more than threshold, good
    BUCKETS = 0.05 # precentage of lower and upper
    THRESHOLD = 0.5 # how much these buckets should have
    THRESHOLD_SIDE = 0.1 # how much right/left bucket should have minimum
    # round up buckets number
    buckets = int(ceil(BUCKETS*(high-low+1)))
    # TODO: in case BUCKETS*len(hist) is not a whole, should we take part
    # of the next bucket as well?
    sides_sum = hist[low:low+buckets].sum() + hist[high-buckets:high+1].sum()

    return (1.0*sides_sum/hist[low:high+1].sum() > THRESHOLD) and\
            (1.0*hist[low:low+buckets].sum()/hist[low:high+1].sum() > THRESHOLD_SIDE) and\
            (1.0*hist[high-buckets:high+1].sum()/hist[low:high+1].sum() > THRESHOLD_SIDE)

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


def iter_rect(x, y, width, height, rect_size, overlap, cover=True):
    """ generate rectangle coordinates inside rectangle
        x, y, width, height
            - dimensions of enclosing rectangle
        rect_size (width, height)
            - size of small innner rectangles
        overlap (width, height)
            - how much should small rectangles overlap
        cover
            - whether to cover whole area. whether to add additional inner
            rectangles in case they don't cover the whole outer rectangle
            (with the specified overlap)
    """
    print(rect_size)
    print(overlap)
    cols = range(x, x+width -rect_size[0]+1, rect_size[0]-overlap[0])
    rows = range(y, y+height-rect_size[1]+1, rect_size[1]-overlap[1])
    if cover:
        # if width isn't a multiple of rect_size[0], we didn't check last column
        if width%(rect_size[0]-overlap[0]):
            cols.append(x+width-rect_size[0])
        # same for height and last row
        if height%(rect_size[1]-overlap[1]):
            rows.append(y+height-rect_size[1])

    for row in rows:
        for col in cols:
            yield col, row, rect_size[0], rect_size[1]

@profile
def hist_iter_rects(img, rect_size, overlap, hist_filter=is_black_white):
    " iterate over good rectangles "
    # TODO: perhaps rewrite with filter()
    it_rects = iter_rect(0, 0, img.shape[1], img.shape[0],
            rect_size, overlap)
    for x, y, width, height in it_rects:
        hist = hist_rect(img, x, y, width, height, max_value=255)
        if hist_filter(hist):
            yield x, y

#################
# ROI histogram #
def roi_hist_ex(channel, rect_size, rect_overlap, hist_filter=is_black_white):
    " return ROI where hist_filter() returns true "
    roi = np.zeros(channel.shape, dtype=np.uint8)
    good_rects = hist_iter_rects(channel, rect_size, rect_overlap, hist_filter=hist_filter)
    for x, y in good_rects:
        pts = [[x, y], [x+rect_size[0], y], [x+rect_size[0], y+rect_size[1]], [x, y+rect_size[1]]]
        roi[y:y+rect_size[1], x:x+rect_size[0]] = np.ones((rect_size[1], rect_size[0]))

    return roi
'''
def roi_hist(img, channel=1, hist_filter=is_black_white,
            resize=INPUT_WIDTH, # None to not resize
            rect_size=(RECT_SIZE_X, RECT_SIZE_Y), #
            overlap=(RECT_OVERLAP_X, RECT_OVERLAP_X)):
    " return ROI where hist_filter() returns true "

    original_shape = img.shape
    if resize is not None:
        # resize
        ratio = 1.0*resize/max(img.shape[:2])
        img = cv2.resize(img, (0,0), fx=ratio, fy=ratio)

    # calculate small roi
    roi = roi_hist_ex(img, rect_size, rect_overlap, hist_filter)
    if resize is not None:
        # resize roi
        roi = cv2.resize(roi, original_shape[::-1])
    return roi
'''
###########################
# ROI enclosing rectangle #
def find_roi(img, channel=1, rect_size=(RECT_SIZE_X, RECT_SIZE_Y),
             overlap=(RECT_OVERLAP_X, RECT_OVERLAP_Y)):
    '''
    Return region of interest as tuple ((x_min, y_min), (x_max, y_max)).
    '''
    img = get_light(img, channel=channel)
    points = []
    for x, y in hist_iter_rects(img, rect_size, overlap):
        points.append((x, y))
        points.append((x+rect_size[0], y+rect_size[1]))
    points = np.array(points)
    if len(points) == 0:
        return None
    min = (np.min(points[:, 0]), np.min(points[:, 1]))
    max = (np.max(points[:, 0]), np.max(points[:, 1]))

    return (min, max)

if __name__ == '__main__':
    import sys
    import os
    import argparse
    import time

    parser = argparse.ArgumentParser(description='Find balldrop target using histogram')
    parser.add_argument('images', nargs='+', help='input images')
    parser.add_argument('--height', default=1.8, type=float, help='height from which we took the photo')
    parser.add_argument('--camera', default='iphone 6 plus rect', help='camera that took the photo')
    
    parser.add_argument('--show', action='store_true', help='show results')
    parser.add_argument('--channel', '-c', default=1, type=int, help='channel (HLS) number to use')
    parser.add_argument('--verbose', '-v', action='count')
    parser.add_argument('--benchmark', action='store_true', help='run some benchmarks instead')

    args = parser.parse_args()
    finder = HistogramFind(resize=500, channel=args.channel)
    for img_path in args.images:
        # open image
        img = cv2.imread(img_path)
        if img is None:
            print "Could not open file %s" % (img_path,)
            continue

        t = time.time()

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

        cam_height = args.height # roees height
        try:
            cam = Camera(args.camera)
        except:
            print 'Could not use camera %r' % (args.camera,)
            print 'Available cameras:', Camera.get_cameras()
            sys.exit(1)

        what = 'find_roi_contours'
        #what = 'find_roi'
        if what == 'find_roi_contours':
            # find_roi_contours
            contours = finder.find_roi_contours(img, cam_height, cam)
            print 'contours type:', type(contours), 'len:', len(contours)
            print 'contours:'
            for i, cont in enumerate(contours):
                print i, 'area:', cv2.contourArea(cont), 
                rect = cv2.boundingRect(cont)
                print 'rect:', rect
                cv2.rectangle(img, rect[0:2], (rect[0]+rect[2], rect[1]+rect[3]), (255, 0,0), 5)
            cv2.drawContours(img, contours, -1, (0, 0, 255), 5)
        elif what == 'find_roi':
            # find_roi
            rect = finder.find_roi(img, cam_height, cam)
            if rect is None:
                print 'rect type:', type(rect)
            else:
                print 'rect type:', type(rect), 'rect:', len(rect)
            corner_min, corner_max = rect
            #corner_min, corner_max = finder.find_roi(img, cam_height, cam)
            int_ceil = lambda _: int(ceil(_))
            corner_min, corner_max = tuple(map(int_ceil, corner_min)), tuple(map(int_ceil, corner_max))
            print 'rect:', corner_min, corner_max
            cv2.rectangle(img, corner_min, corner_max, (255, 0,0), 5)

        if args.verbose > 0:
            print '%fs for %s' % (time.time()-t, img_path)
        if args.show:
            debug_image(img)

