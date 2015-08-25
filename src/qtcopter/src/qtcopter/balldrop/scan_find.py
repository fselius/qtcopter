#!/usr/bin/env python2

'''
Find the target from contour detection and the contours nesting.

Usage:
    ./ransac_find.py <image>
'''

import cv2
import cv
import numpy as np
import random
import argparse
from ptgrey import PTGreyCamera
import flycapture2 as fc2
from time import time
from math import sin, cos, pi
import math

def show_img(img, wait=True, title='bah'):
    while max(img.shape[:2]) > 500:
        img = cv2.pyrDown(img)
    #ratio = 500./max(img.shape)
    #cv2.resize(img, 

    cv2.imshow(title, img)
    if cv2.waitKey(1)&0xff == ord('q'):
        raise KeyboardInterrupt()
    if wait:
        while True:
            if cv2.waitKey(0)&0xff==ord('q'):
                break
        cv2.destroyAllWindows()

class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
    def __repr__(self):
        return '<Circle (%.2f, %.2f) %.2f>' % (self.center[0], self.center[1], self.radius)

class ScanFind:
    def __init__(self, center_black, number_of_rings, debug=True):
        self._center_black = center_black
        self._number_of_rings = number_of_rings
        self._canny_threshold1 = 200
        self._canny_threshold2 = 20
        self._debug = debug
        self._min_radius = 20
        #self._max_radius = max(320)
        self._scan_jump = self._min_radius/4.
        self._near_threshold = self._scan_jump*4
    @staticmethod
    def _dist(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    def _near(self, a, b):
        return self._dist(a, b)<self._near_threshold
    def find_target(self, image):
        '''
        Return the center of the target in pixel coordinates as tuple (x, y).
        '''
        # resize input image, assuming target takes at least 1/2 of the frame.
        self.resize = 400
        orig_shape = image.shape[:2]
        while min(image.shape[:2]) > self.resize:
            image = cv2.pyrDown(image)
        ratio = 1.*image.shape[0]/orig_shape[0]

        center, radius = self.find_circles(image)
        if center is None:
            return (None, None)
        return (center[0]/ratio, center[1]/ratio), radius/ratio

    def find_circles(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blur kernel about 20 for 1000x1000px, 10 for 500x500, 5 for 250x250
        blurk = max(image.shape)/50
        if ~blurk&1:
            blurk += 1
        image = cv2.medianBlur(image, blurk)

        can = cv2.Canny(image, self._canny_threshold1, self._canny_threshold2)

        if self._debug:
            show_img(can, wait=False, title='canny')

        # get distance matrix (distance to nearest 1)
        distance = cv2.distanceTransform(255-can, cv.CV_DIST_L2, 3)
        points = [_[::-1] for _ in np.transpose(can.nonzero())]
        if len(points) == 0:
            return None, None

        # find horizontal and vertical lines
        circles_hor = self.find_lines(distance, image)
        circles_ver = self.find_lines(distance, image, transpose=True)
        show_img(image, wait=False, title='debug2?')

        # TODO: group by distance of centers also according to size of circle
        # groupify horizontal found circles
        groups_hor = self.groupify(circles_hor)
        circles_hor = []
        for group in groups_hor:
            center = np.mean([c.center for c in group], axis=0)
            radius = np.mean([c.radius for c in group])
            circles_hor.append(Circle(center, radius))

        # groupify horizontal found circles
        groups_ver = self.groupify(circles_ver)
        circles_ver = []
        for group in groups_ver:
            center = np.mean([c.center for c in group], axis=0)
            radius = np.mean([c.radius for c in group])
            circles_ver.append(Circle(center, radius))

        # find circles that are both in horizontal and vertical scan find
        good_circles = []
        for circle in circles_hor:
            near = filter(lambda c: self._near(c.center, circle.center), circles_ver)
            if len(near) == 0:
                continue
            
            group = [circle] + near
            center = np.mean([c.center for c in group], axis=0)
            radius = np.mean([c.radius for c in group])
            good_circles.append(Circle(center, radius))

        if len(good_circles) > 1:
            print 'WARNING: multiple circles..'
            

        if self._debug:
            for circle in good_circles:
                center = circle.center
                radius = circle.radius
                cv2.circle(image, tuple(map(int, center)), 4, 255, -1)
                cv2.circle(image, tuple(map(int, center)), int(radius), 255, 1)
            show_img(image, wait=False, title='debug2?')

        if len(good_circles) > 0:
            return good_circles[0].center, good_circles[0].radius 
        return (None, None)

    def groupify(self, circles):
        circles = set(circles)

        groups = []
        while len(circles) > 0:
            before_len = len(circles)
            for group in groups:
                right = max(group, key=lambda c: c.center[1])
                bottom = max(group, key=lambda c: c.center[0])
                last = set([right, bottom])
                close = set()
                for circle in last:
                    close |= set(filter(lambda c: self._near(c.center, circle.center), circles))
                group |= close
                circles -= close
            if before_len == len(circles):
                # we didn't add any circles to the groups. create new group.
                group = set([circles.pop()])
                groups.append(group)
                circles -= group

        return groups

            



    def find_lines(self, distance, image, transpose=False):
        if transpose:
            distance = distance.transpose()
        circles = []
        count = np.ceil(distance.shape[0]/self._scan_jump)
        for y in np.linspace(0, distance.shape[0]-1, num=count):
            xx = np.arange(distance.shape[1])
            yy = np.tile(y, len(xx)).astype(np.int)
            z = distance[yy, xx]

            # indexes of borders
            zi = (z==0).nonzero()[0]

            # distances between borders
            d = zi[1:]-zi[:-1]
            di = zi[:-1] # indexes of distances

            # remove intervals of length 1 (for when we catch border multiple times)
            d, di = d[d!=1], di[d!=1]

            d = d.astype(np.float)
            #print 'di, z, d:', di, z, d

            a0, a1, a2, a3, a4 = d[:-4], d[1:-3], d[2:-2], d[3:-1],d[4:]
            avg = (a0+a1+a2+a3+a4)/6 # (a4 (a0) is twice the length)
            #print 'avg:', avg, 'low/high:', avg*0.8, avg*1.2

            # find 5 consecutive values that are around the average of the 5 values
            p = 1.2
            # ratio 1: 1/1/1/1/2 (outer to inner ring)
            r1 = (1/p<a0/avg) & (a0/avg<p) &\
                 (1/p<a1/avg) & (a1/avg<p) &\
                 (1/p<a2/avg) & (a2/avg<p) &\
                 (1/p<a3/avg) & (a3/avg<p) &\
                 (1/p<a4/2/avg) & (a4/2/avg<p)

            # ratio 2: 2/1/1/1/1 (inner to outer ring)
            r2 = (1/p<a0/2/avg) & (a0/2/avg<p) &\
                 (1/p<a1/avg) & (a1/avg<p) &\
                 (1/p<a2/avg) & (a2/avg<p) &\
                 (1/p<a3/avg) & (a3/avg<p) &\
                 (1/p<a4/avg) & (a4/avg<p)

            for i in r1.nonzero()[0]:
                start = di[i]
                radius = avg[i]*5
                # TODO: what's the best estimate for the center?
                center = (start+radius, y)
                line_end = (int(start), int(y))
                if transpose:
                    center = center[::-1]
                    line_end = tuple(line_end[::-1])

                circles.append(Circle(center, radius))

                cv2.line(image, tuple(map(int, center)), line_end, 255, 1)
                cv2.circle(image, tuple(map(int, center)), 2, 255, -1)

            for i in r2.nonzero()[0]:
                end = di[i+4]+d[i+4]
                radius = avg[i]*5
                center = (end-radius, y)
                line_end = (int(end), int(y))
                if transpose:
                    center = center[::-1]
                    line_end = tuple(line_end[::-1])

                circles.append(Circle(center, radius))

                cv2.line(image, tuple(map(int, center)), line_end, (255, 0, 0), 1)
                cv2.circle(image, tuple(map(int, center)), 2, (255, 0, 0), -1)

        if self._debug:
            show_img(image, wait=False, title='debug2?')
        return circles

    def check_circles(self, distance, center, img):
        count = 50.
        radiuses = np.array([], dtype=float)
        for alpha in np.linspace(0, 2*pi, num=count, endpoint=False):
            rad = self.check_circles_line(distance, center, alpha, img)
            if rad is not None:
                radiuses = np.append(radiuses, rad)
        #print 'radiuses:', radiuses
        # TODO: check if radiuses are mostly same
        # NOTE: check if radiuses are roughly the same will work if the checked
        # center is almost correct. however, what if we miss the center slightly?
        # hm.. then we should have at least 2 directions correct (if we missed the center
        # a bit up, then still up/down directions should yield a line with reoccuring radius distances
        # but left/right will be quite wrong.) what to do?
        good = (0.8*np.median(radiuses)<radiuses)&(radiuses<1.2*np.median(radiuses))
        good = radiuses[good]
        print 'count:', count, 'radiuses:', len(radiuses), 'good:', len(good), 'med:', np.median(radiuses), 'avg:', np.average(radiuses)
         
        if len(good) > 0.25*count:
            return center, np.median(radiuses)
        return None, None

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--shutter', default=10, type=float, help='shutter time (10 ms default for indoor, 1-2 ms is fine for outside)')
    parser.add_argument('--gain', default=0, type=float, help='gain')
    parser.add_argument('--debug', action='store_true', help='debug')
    parser.add_argument('-t', '--threshold', default=100, help='')
    parser.add_argument('cam', nargs='*', help='image file or camera')

    args = parser.parse_args()

    if args.cam:
        import cProfile
        finder = ScanFind(True, 3, debug=args.debug)
        for image in args.cam:
            image = cv2.imread(image)
            #center, size = finder.find_target(image)
            cProfile.run("center, size = finder.find_target(image)")
            print 'center:', center
            if center is not None:
                cv2.circle(image, tuple(map(int, center)), 10, (0, 255, 255), -1)
                cv2.circle(image, tuple(map(int, center)), int(size/2), (0, 255, 255), 3)
                show_img(image)
            else:
                print 'could not find target'
                show_img(image)
    else:
        c = PTGreyCamera()
        # set manual values
        c.set_property_manual(fc2.AUTO_EXPOSURE, 0) # exposure = 0, we don't modify this. I'm not sure, but it had no effect.
        c.set_property_manual(fc2.SHUTTER, args.shutter) # 10ms shutter (1/100, hopefully fast enough)
        # if frame_rate is too high, it is set to maximum :)
        c.set_property_manual(fc2.FRAME_RATE, 100) # maximum framerate
        c.set_property_manual(fc2.GAIN, args.gain)
        c.print_infos()

        c.start_capture()
        finder = ScanFind(True, 3, debug=args.debug)
        
        try:
            while True:
                img = c.get_frame()
                t = time()
                center, size = finder.find_target(img)
                t = time()-t
                print 'time:', t, 1/t, 'fps'
                if center is not None:
                    cv2.circle(img, (int(center[0]), int(center[1])), 10, (255, 255, 0), -1)
                show_img(img, wait=False, title='img_target')
        except KeyboardInterrupt:
            print 'bye..'
