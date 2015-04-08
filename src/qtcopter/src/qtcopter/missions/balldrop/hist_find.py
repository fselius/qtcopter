#!/usr/bin/env python

import sys
import cv2
import pylab
import time


img_path = './IMG_0527.JPG'
channel = 0
if len(sys.argv) > 1:
	img_path = sys.argv[1]
if len(sys.argv) > 2:
	channel = int(sys.argv[2])





img = cv2.imread(img_path)
t= time.time()
img =  cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
# resize
#img = cv2.
channels = cv2.split(img)
channel = channels[channel]
channel = cv2.equalizeHist(channel)
hist = cv2.calcHist([channel], [0], None, [32], [0,256])

print hist
print hist[0], hist[-1], (1.0*hist[0]+hist[-1])/sum(hist)
print 'time:', time.time()-t
pylab.plot(hist)
pylab.show()

