#!/usr/bin/env python2

'''
Find the target from contour detection and the contours nesting.

Usage:
    ./polarity_find.py <image>
'''

import cv2
import cv
import numpy as np
import random
import argparse
from ptgrey import PTGreyCamera
import flycapture2 as fc2
from time import time

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

class RANSACFind:
    def __init__(self, center_black, number_of_rings, debug=True):
        self._center_black = center_black
        self._number_of_rings = number_of_rings
        self._debug = debug
        self._canny_threshold1 = 200
        self._canny_threshold2 = 20

        self._max_iterations = 1000
        #self._max_proc_time = 2 # seconds

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
            print can.shape, 'points:', can.sum()/255

        if self._debug:
            show_img(can, wait=False, title='canny')

        circles = [] # (center, radius, percentage)
        MAX_CIRCLES = 10
        # get distance matrix (distance to nearest 1)
        distance = cv2.distanceTransform(255-can, cv.CV_DIST_L2, 3)
        points = [_[::-1] for _ in np.transpose(can.nonzero())]
        if len(points) == 0:
            return None, None
        MIN_RADIUS = 20
        MAX_RADIUS = max(can.shape)
        for i in xrange(self._max_iterations):
            # get 3 points for circle
            p = [random.randrange(len(points)) for _ in range(3)]
            # make sure we get different points
            if len(set(p)) != 3:
                continue
            rand_points = [points[_] for _ in p]

            # make a circle from the points
            center, radius = self.get_circle(rand_points)
            # if radius is None, there was an error (points on one line)
            if radius is None:
                continue
            # throw too big / too small circles
            if radius < MIN_RADIUS or radius > MAX_RADIUS:
                continue
            #centers[j] = center
            #j+=1
            #continue
            percent = self.verify_circle(distance, center, radius)
            if len(circles) == 0 or percent > circles[0][2]:
                circles.append((center, radius, percent))
                circles.sort(key=lambda _: _[2])
                if len(circles) > MAX_CIRCLES:
                    circles = circles[1:]
            if circles[-1][2] > 0.9:
                print 'Break!! we found a circle > 0.9! i=', i
                break
        #centers_x = [_[0] for _ in centers[:j]]
        #centers_y = [_[1] for _ in centers[:j]]
        #x = (np.median(centers_x), np.median(centers_y)), 10
        #print 'average of', j, x
        #return x
        if len(circles) == 0 or sum([_[2] for _ in circles])==0:
            return None, None
        # average by value * percent**2
        avg_x = sum([_[0][0]*_[2]**2 for _ in circles])/sum([_[2]**2 for _ in circles])
        avg_y = sum([_[0][1]*_[2]**2 for _ in circles])/sum([_[2]**2 for _ in circles])
        avg_radius = sum([_[1]*_[2]**2 for _ in circles])/sum([_[2]**2 for _ in circles])
        avg_center = (avg_x, avg_y)
        for (center, radius, percent) in circles:
            #print percent, radius, center
            cv2.circle(image, tuple(map(int, center)), int(radius), (255, 0, 0), 1)
        cv2.circle(image, tuple(map(int, circles[-1][0])), int(circles[-1][1]), (255, 0, 0), 3)
        self.verify_circle(distance, circles[-1][0], circles[-1][1])
        cv2.circle(image, tuple(map(int, avg_center)), 5, (255, 0, 0), -1)
        if self._debug:
            show_img(image, wait=False, title='debug2?')
        # TODO: find the actual radius. lol.
        return (avg_center, avg_radius)

    @staticmethod
    def get_circle(p):
        # http://mathforum.org/library/drmath/view/55239.html
        x = np.linalg.det([
                    [p[0][0]**2+p[0][1]**2, p[0][1], 1],
                    [p[1][0]**2+p[1][1]**2, p[1][1], 1],
                    [p[2][0]**2+p[2][1]**2, p[2][1], 1],
            ])
        y = np.linalg.det([
                    [p[0][0], p[0][0]**2+p[0][1]**2, 1],
                    [p[1][0], p[1][0]**2+p[1][1]**2, 1],
                    [p[2][0], p[2][0]**2+p[2][1]**2, 1],
            ])
        bah = 2*np.linalg.det([
                    [p[0][0], p[0][1], 1], 
                    [p[1][0], p[1][1], 1], 
                    [p[2][0], p[2][1], 1], 
            ])
        if bah == 0:
            return None, None
        x /= bah
        y /= bah

        radius = np.linalg.norm(p[0]-(x,y))
        return (x, y), radius
    @staticmethod
    def verify_circle(distance, center, radius, debug=False):
        max_dist = min(radius/25, 50)
        count = 0
        inlie = 0
        #incircle = 0
        for a in np.linspace(0, 2*np.pi, 50):
            count += 1
            x, y = center[0] + np.cos(a) * radius, center[1]-np.sin(a)*radius;
            if x<0 or x>=distance.shape[1] or y<0 or y>=distance.shape[0]:
                continue
            #incircle += 1
            if debug:
                print x, y, distance[y, x]
            #inlie += distance[y, x]
            if distance[y, x] < max_dist:
                inlie += 1
        #if incircle == 0:
        #    print center, radius
        #    return 0
        #return -1.0*inlie/incircle
        return 1.0*inlie/count

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--shutter', default=10, type=int, help='shutter time (10 ms default for indoor, 1-2 ms is fine for outside)')
    parser.add_argument('--gain', default=0, type=float, help='gain')
    parser.add_argument('--debug', action='store_true', help='debug')
    parser.add_argument('-t', '--threshold', default=100, help='')
    parser.add_argument('cam', nargs='*', help='image file or camera')

    args = parser.parse_args()

    if args.cam:
        import cProfile
        finder = PolarityFind(True, 3, args.threshold, debug=args.debug)
        for image in args.cam:
            image = cv2.imread(image)
            #center, size = finder.find_target(image)
            cProfile.run("center, size = finder.find_target(image)")
            print 'center:', center
            if center is not None:
                cv2.circle(image, tuple(map(int, center)), 10, (0, 255, 255), -1)
                show_img(image)
            else:
                print 'could not find target'
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
        finder = RANSACFind(True, 3, debug=args.debug)
        
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
