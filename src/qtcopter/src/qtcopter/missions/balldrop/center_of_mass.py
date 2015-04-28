#!/usr/bin/env python

#import cv
#import cv2
import numpy as np
#from numpy.linalg import norm as norm
from scipy.ndimage.measurements import center_of_mass as com

def center_of_mass(roi):
    height = len(img[0])
    weight = len(img)
    pixels = height*weight
    cmx = 0
    cmy = 0

    #CM = com(numpy.array(roi))
    CM = com(roi)

#    for x in xrange(0,weight):
#        for y in xrange(0,height):
#            if img[x][y] == 1:
#                cmx += x
#                cmy += y
#
#cmx = cmx/pixels
#cmy = cmy/pixels
#
#return cmx, cmy

return CM
