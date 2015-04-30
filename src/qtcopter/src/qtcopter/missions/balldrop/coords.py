#!/usr/bin/env python

import argparse
import numpy as np

def cam_pixel_to_xy(distance, width_px, height_px, x_px, y_px, x_offset_m, y_offset_m):#, focal_length):
    '''
    cam_pixel_to_xy - convert camera pixel offset to ground xy offset relative
                      view direction.

    distance               - distance to ground [meters]
    width_px, height_px    - picture size [pixels]
    x_px, y_px             - image point [pixels]
    x_offset_m, y_offset_m - offset of camera from balldrop on quadcopter in meters
    '''
    (xx, yy) = (2.*x_px*distance/width_px, 2.*y_px*distance/height_px)
    #(x, y)   = ((distance/focal_length)*(x_pixel, y_pixel)
    #print "dx = %d dy = %d, dz = %d\n" % (x, y, distance)
    #print "dx = %f dy = %f, dz = %f\n" % (xx-x_os, yy-y_os, distance)
    #return x, y
    return (xx-x_offset_m, yy-y_offset_m)

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Find balldrop target location')
    parser.add_argument('--distance', '-d', default=1, type=float, help='Z axis distance to ground')
    parser.add_argument('--pixel_width', '-w', default=1, type=int, help='width of the image in pixels')
    parser.add_argument('--pixel_height', '-hh', default=1, type=int, help='height of the image in pixels')
    parser.add_argument('--x_pixel', '-x', default=1, type=int, help='location in image in pixels')
    parser.add_argument('--y_pixel', '-y', default=1, type=int, help='location in image in pixels')
    parser.add_argument('--x_offset', '-xos', default=0, type=float, help='location of objects offset from the camera')
    parser.add_argument('--y_offset', '-yos', default=0, type=float, help='location of object offset from the camera')



    args = parser.parse_args()

    distance     = args.distance
    pixel_width  = args.pixel_width
    pixel_height = args.pixel_height
    x_pixel      = args.x_pixel
    y_pixel      = args.y_pixel
    x_os         = args.x_offset
    y_os         = args.y_offset

    print cam_pixel_to_xy(distance, pixel_width, pixel_height, x_pixel, y_pixel, x_os, y_os)            

