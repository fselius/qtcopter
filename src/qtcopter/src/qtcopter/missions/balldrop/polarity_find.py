#!/usr/bin/env python2

'''
Find the target from contour detection and the contours nesting.

Usage:
    ./polarity_find.py <image>
'''

import cv2
import numpy as np
import sys
from random import randint as rand

if __name__ == '__main__':
    image = cv2.imread(sys.argv[1])

    h, w = image.shape[:2]
    img = cv2.blur(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (3, 3))
    _, threshold = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
    contours, [hierarchy] = cv2.findContours(threshold, cv2.RETR_TREE,
                                             cv2.CHAIN_APPROX_SIMPLE)

    assert(len(hierarchy) == len(contours))

    def count_children(hierarchy, i):
        if i <= -1:
            return 0
        count = 0
        while i > -1:
            count += count_children(hierarchy, hierarchy[i][2])
            i = hierarchy[i][0]
        return count + 1

    # Only keep top-level contours with min 2 children and at least 50 points
    contours_idx = [i for i, c in enumerate(contours) if len(c) >= 50 and
                    hierarchy[i][2] > -1 and hierarchy[i][3] <= -1 and
                    count_children(hierarchy, hierarchy[i][2]) >= 2]

    def flatten_hierarchy(contours, hierarchy, initial):
        contours_idx = [hierarchy[initial][2]]
        ret = [initial]
        while len(contours_idx) > 0:
            i = contours_idx.pop()
            if i <= -1:
                continue
            if len(contours[i]) >= 50:
                ret.append(i)
                contours_idx.append(hierarchy[i][2])
            contours_idx.append(hierarchy[i][0])
        return ret

    contours_idx = sum([flatten_hierarchy(contours, hierarchy, i)
                       for i in contours_idx], [])

    ellipses = [cv2.fitEllipse(contours[i]) for i in contours_idx]

    cv2.namedWindow('contours', cv2.WINDOW_AUTOSIZE)
    for i, j in enumerate(contours_idx):
        color = (rand(0, 255), rand(0, 255), rand(0, 255))
        #cv2.drawContours(image, contours, j, color, 1, 8)
        cv2.ellipse(image, ellipses[i], color, 2, 8)
    cv2.imshow('contours', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
