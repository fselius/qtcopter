#!/usr/bin/env python2

'''
Find the target from contour detection and the contours nesting.

Usage:
    ./polarity_find.py <image>
'''

import cv2
import numpy as np


class PolarityFind:
    def __init__(self, center_black, number_of_rings, threshold, debug=False):
        self._center_black = center_black
        self._number_of_rings = number_of_rings
        self._debug = debug
        self._threshold = threshold

    def find_contours(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # blur kernel about 20 for 1000x1000px, 10 for 500x500, 5 for 250x250
        blurk = max(image.shape)/50
        if ~blurk&1:
            blurk += 1
        image = cv2.medianBlur(image, blurk)

        _, threshold = cv2.threshold(image, self._threshold, 1, cv2.THRESH_BINARY)

        if threshold is None or threshold.size == 0:
            return None, None

        # FIXME: the outer rectangle of the ROI is detected as contour.. what do do about that?
        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        if hierarchy is not None:
            hierarchy = hierarchy[0]

        return contours, hierarchy

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

        contours, hierarchy = self.find_contours(image)
        if contours is None or hierarchy is None:
            return (None, None)

        contours_idx = [i for i, c in enumerate(contours) if len(c) >= 40]

        # Find leaf contours with parent and at least 50 points.
        # TODO: This doesn't work correctly. Too many false negatives..
        # Also, looks like this isn't a terrible performance issue to go over
        # all the contours.
        leaf_contours_idx = [i for i in contours_idx if len(contours[i]) >= 4 and
                             hierarchy[i][2] >= 0 and hierarchy[i][3] < 0]

        # DEBUG OUTPUT
        if self._debug:
            # TODO/FIXME:
            # this draws the contours on image, which is a partial view of the
            # original image. Perhaps this doesn't work correclty? or maybe it's
            # too slow?
            cv2.drawContours(image, contours, -1, (0, 0, 255), 5)
            cv2.drawContours(image, [contours[i] for i in leaf_contours_idx], -1, (0, 255, 0), 4)
            cv2.drawContours(image, [contours[i] for i in contours_idx], -1, (255, 0, 0), 3)
            show_img(image)

        # Go up and check polarity
        # FIXME: Until leaf_contours_idx is fixed, go over all the contours.
        #for i in leaf_contours_idx:
        for i in range(len(contours)):
            polarity = self._center_black
            inner_counter = 0
            idx = i
            while idx >= 0:
                area = cv2.contourArea(contours[idx], True)

                if area > 0 and polarity or area < 0 and not polarity:
                    inner_counter += 1
                    polarity = not polarity
                #else:
                #    # Go to next in same hierarchy level
                #    idx = hierarchy[idx][0]
                #    continue

                if inner_counter > 3:
                    # TODO/FIXME: This returns the diameter of the inner white circle!
                    # TODO: check all circles (parents)
                    moments = cv2.moments(contours[idx])
                    x = int(moments['m10']/moments['m00'])
                    y = int(moments['m01']/moments['m00'])
                    r = np.sqrt(4*cv2.contourArea(contours[idx])/np.pi)
                    r = 5./2 * r # circle / inner white circle radius ratio
                    #return ((x, y), r)
                    return ((x/ratio, y/ratio), r/ratio)
                idx = hierarchy[idx][2]

        return (None, None)

def show_img(img):
    cv2.imshow('model', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    import sys
    image = cv2.imread(sys.argv[1])

    import cProfile
    finder = PolarityFind(True, 3, 100, debug=True)
    center, size = finder.find_target(image)
    #cProfile.run("center, size = finder.find_target(image)")
    print 'center:', center
    if center is not None:
        cv2.circle(image, tuple(map(int, center)), 5, (0, 255, 255), -1)
        show_img(image)
    else:
        print 'could not find target'
