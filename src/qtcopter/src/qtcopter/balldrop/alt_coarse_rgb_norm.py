import cv
import cv2
import numpy as np
from numpy.linalg import norm as norm


def rgb_dist(img, height):
    img = cv2.imread('img')

    height = len(img[0])
    weight = len(img)
    pixels = height*weight

    imgr = cv2.resize(img, (200,200))

    rgb_dist = np.zeros((200,200))
    
    imgr = imgr.astype(np.uint16)
    
    for x in xrange(0,200):
        for y in xrange(0,200):
            rgb_dist[x][y] = norm(imgr[x][y])
                
    cmx =0
    cmy = 0
    counter = 0
    for x in xrange(0,200):
        for y in xrange(0,200):
            if rgb_dist[x][y] > 435:
                cmx += x
                cmy += y
                counter +=1
                
    cmy = cmy/counter
    cmx = cmx/counter

#create roi matrix in original sizes
#update cmy and cmx to original image sizes
#update roi matrix in a radius based on target size and drone height



#    for x in xrange(0,200):
#        for y in xrange(0,200):
#            if rgb_dist[x][y] < 435:
#                imgr[x][y] = [0,0,0];
