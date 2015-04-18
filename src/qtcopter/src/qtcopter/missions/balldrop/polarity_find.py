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
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    contours = [c for c in contours if len(c) > 5]
    minEllipse = [cv2.fitEllipse(c) for c in contours]

    cv2.namedWindow('contours', cv2.WINDOW_AUTOSIZE)
    vis = np.zeros((h, w, 3), np.uint8)
    for i in range(0, len(contours)):
        color = (rand(0, 255), rand(0, 255), rand(0, 255))
        cv2.drawContours(vis, contours, i, color, 1, 8)
        cv2.ellipse(vis, minEllipse[i], color, 2, 8)
    cv2.imshow('contours', vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
