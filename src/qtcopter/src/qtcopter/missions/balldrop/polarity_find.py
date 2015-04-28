#!/usr/bin/env python2

'''
Find the target from contour detection and the contours nesting.

Usage:
    ./polarity_find.py <image>
'''

import cv2
import numpy as np
import sys
from math import sin, cos
import rospy


def find_contours(image):
    h, w = image.shape[:2]
    img = cv2.blur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3))
    _, threshold = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
    contours, [hierarchy] = cv2.findContours(threshold, cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)

    assert(len(hierarchy) == len(contours))
    return contours, hierarchy


def count_children(hierarchy, i):
    if i <= -1:
        return 0
    count = 0
    while i > -1:
        cnt = count_children(hierarchy, hierarchy[i][2])
        if cnt > count:
            count = cnt
        i = hierarchy[i][0]
    return count + 1


def find_target(image):
    '''
    Return the center of the target in pixel coordinates as tuple (x, y).
    '''
    contours, hierarchy = find_contours(image)

    # Find leaf contours with parent and at least 50 points.
    leaf_contours_idx = [i for i, c in enumerate(contours) if len(c) >= 50 and
                         hierarchy[i][2] >= 0 and hierarchy[i][3] < 0]

    # Go up and check polarity
    for i in leaf_contours_idx:
        polarity = True
        inner_counter = 0
        idx = i
        while idx >= 0:
            cv2.drawContours(image, contours, idx, (0, 0, 255))
            area = cv2.contourArea(contours[idx], True)

            if area > 0 and polarity or area < 0 and not polarity:
                inner_counter += 1
                polarity = not polarity
            #else:
            #    break

            if inner_counter > 3:
                rospy.loginfo('Found circle contour.')
                # TODO: check all circles (parents)
                moments = cv2.moments(contours[idx])
                x = int(moments['m10']/moments['m00'])
                y = int(moments['m01']/moments['m00'])
                return (x, y)
            idx = hierarchy[idx][2]

    return None

if __name__ == '__main__':
    image = cv2.imread(sys.argv[1])

    center = find_target(image)

    cv2.circle(image, center, 5, (0, 0, 255), 5)
    cv2.imshow('model', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
