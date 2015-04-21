#!/usr/bin/env python

import argparse
import numpy as np

def find_coords(distance, pixel_width, pixel_height, x_pixel, y_pixel):#, focal_length):
    (xx, yy) = (2*x_pixel*distance/pixel_width, 2*y_pixel*distance/pixel_height)
    #(x, y)   = ((distance/focal_length)*(x_pixel, y_pixel)
    #print "dx = %d dy = %d, dz = %d\n" % (x, y, distance)
    print "dx = %d dy = %d, dz = %d\n" % (xx, yy, distance)
    #return x, y
    return (xx-x_os, yy-y_os)

def main();
    parser = argparse.ArgumentParser(description='Find balldrop target location')
    parser.add_argument('--distance', '-d', default=1, type=float, help='Z axis distance to ground')
    parser.add_argument('--pixel_width', '-w', default=1, type=int, help='width of the image in pixels')
    parser.add_argument('--pixel_height', '-h', default=1, type=int, help='height of the image in pixels')
    parser.add_argument('--x_pixel', '-x', default=1, type=int, help='location in image in pixels')
    parser.add_argument('--x_pixel', '-y', default=1, type=int, help='location in image in pixels')

    args = parser.parse_args()

    distance     = args.distance
    pixel_width  = args.pixel_width
    pixel_height = args.pixel_height
    x_pixel      = args.x_pixel
    y_pixel      = args.y_pixel

    print find_coords(distance, pixel_width, pixel_height, x_pixel, y_pixel)            

if __name__=='__main__':
    main()
