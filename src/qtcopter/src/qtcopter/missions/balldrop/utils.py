#!/usr/bin/env python

import math
import numpy as np

#import cv
import cv2

#from numpy.linalg import norm as norm
from scipy.ndimage.measurements import center_of_mass 
from qtcopter.navigation.Camera import camera

TARGET_A4_RADIUS = 0.21/2 # A4 paper meters
TARGET_MISSION_RADIUS = 1 # meters

TARGET_RADIUS = TARGET_A4_RADIUS # meters
TARGET_CIRCLE_AREA = math.pi * TARGET_RADIUS**2 # circle area
TARGET_SQUARE_AREA = (TARGET_RADIUS*2)**2 #


def contour_size_ok(contour, distance, camera=camera):
    ''' contour_size_ok - determine if contour size is ok for height '''
    #cv2.contourArea(cont) # -> area
    #cv2.boundingRect(cont) # -> x, y, width, height
    
    # find enclosing rectangle
    x, y, width, height = cv2.boundingRect(contour)
    # top_left, top_right, bottom_left, bottom_right
    corners = [(x, y), (x+width, y), (x, y+height), (x+width, y+height)] 
    # convert to ground offsets in meters
    corners = map(lambda _: camera.get_ground_offset(_, distance, True), corners)

    area = cv2.norm(corners[0], corners[1]) * cv2.norm(corners[0], corners[2])
    #print 'area = %f, wanted > %f' % (area, 0.8 * TARGET_CIRCLE_AREA)
    return area > 0.8 * TARGET_CIRCLE_AREA

def filter_contours(contours, distance, camera=camera):
    ''' filter out bad contours  '''
    # filter by size of enclosing rectangle. we only make sure it's big enough.
    contours = filter(lambda x: contour_size_ok(x, distance, camera), contours)
    # perhaps we should also filter by area. this can be useful if we found a
    # rectangle that is for example very big, but too sparse to actually be a
    # target. a target would have a lot of 
    return contours