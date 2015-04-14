#!/usr/bin/env python

import sys
import os
import cv2
import pylab
import time
import numpy as np

# dummy @profile decorator
import __builtin__

try:
    __builtin__.profile
except AttributeError:
    # No line profiler, provide a pass-through version
    def profile(func): return func
    __builtin__.profile = profile



img_path = './IMG_0527.JPG'
channel = 2
if len(sys.argv) < 1:
	print 'Usage: %s <image> [channel (HLS)=2]' % (sys.argv[0],)
	sys.exit(1)
if len(sys.argv) > 1:
	img_path = sys.argv[1]
if len(sys.argv) > 2:
	channel = int(sys.argv[2])

@profile
def find_edges(hist, cut=0.05):
	""" find edges of histogram (for the black-white case). Usually, the
	colors will be either over exposed (white is white, black is gray),
	or under exposed (white is gray, black is black).
	We find actual edges, rather than doing a histogram equalization, which
	isn't exactly what we want, and it takes away the beautiful peaks that
	we would like to find.
	
	hist - histogram
	cut - how much we want to cut of the histogram (cut from each side)
	"""
	
	total = hist.sum()
	#print type(cut), type(total), cut, total
	#print hist[:10]
	cut = np.float32(cut*float(total))
	#cut = cut*total
	# find low edge
	
	sums = hist.cumsum()
	i = np.argmax(sums>cut)
	low = max(0, i-1) # if i < 0..

	i = np.argmax(sums>(total-cut))
	high = min(len(hist)-1, i+1) # if i >= len(hist)

	return low, high

@profile
def is_black_white(hist):
	" decide whether histogram is of black/white pattern "
	low, high = find_edges(hist)	
	#print 'low, high, total hist length:', low, high, len(hist)

	#hist = hist[low:high+1]

	# take lower and upper % buckets (darkest + lightest), if they
	# contain more than threshold, good
	BUCKETS = 0.05 # precentage of lower and upper
	THRESHOLD = 0.5 # how much these buckets should have

	buckets = int(BUCKETS*(high-low+1))
	#buckets = int(BUCKETS*len(hist))
	# TODO: in case BUCKETS*len(hist) is not a whole, should we take part
	# of the next bucket as well?
	sides_sum = hist[low:low+buckets].sum() + hist[high-buckets:high+1].sum()
	#sides_sum = sum(hist[:buckets]) + sum(hist[-buckets:])
	#print 'low:', 1.0*sum(hist[:buckets])/sum(hist), 'high:', 1.0*sum(hist[-buckets:])/sum(hist), 'total:', 1.0*sides_sum/sum(hist)
	#if 1.0*sides_sum/sum(hist) > THRESHOLD:
	return 1.0*sides_sum/hist[low:high+1].sum() > THRESHOLD
	#return 1.0*sides_sum > hist[low:high+1].sum()*THRESHOLD

@profile
def hist_rect(channel, x=0, y=0, width=None, height=None, max_value=255):
	" calculate histogram in rectangle "
	ranges = [0, max_value+1]
	dims = [max_value+1]

	# defaults
	if width is None:
		width = channel.shape[1]-x
	if height is None:
		height = channel.shape[0]-y
	
	# create mask of pixels we want to calc histogram of
	mask = np.zeros(channel.shape, dtype=np.uint8)
	mask[y:y+height, x:x+width] = np.ones((width, height))
	# calc histogram :)
	hist = cv2.calcHist([channel], [0], mask, dims, ranges)
	return hist.reshape(hist.size) # return a flat array

def iter_rect(x, y, width, height, rect_width, rect_height, overlap_x, overlap_y):
	" generate rectangle coordinates "
	i=0
	# TODO: Currently we don't handle cases when right\bottom edge isn't
	# at round boundary (i.e. not multiple of rect_width-overlap_x)
	# we should probably also yield last row/column end the end to check
	# these edges.
	for row in range(x, width-rect_width, rect_width-overlap_x):
		for col in range(y, height-rect_height, rect_height-overlap_y):
			yield row, col, rect_width, rect_height
			i+=1
	print 'loops:', i

@profile
def hist_iter_rects(channel, rect_width, rect_height, overlap_x, overlap_y):
	" iterate over good rectangles "
	# TODO: perhaps rewrite with filter()
	it_rects = iter_rect(0, 0, channel.shape[1], channel.shape[0],
			rect_width, rect_height, overlap_x, overlap_y)
	for x, y, width, height in it_rects:
		hist = hist_rect(channel, x, y, width, height, max_value=255)
		if is_black_white(hist):
			yield x, y

# open image
img = cv2.imread(img_path)
if img is None:
	print "Could not open file %s" % (img_path,)
	sys.exit(-1)

t = time.time()

# resize to have maximum 500px width/height
ratio = 500./max(img.shape[:2])
img = cv2.resize(img, (0,0), fx=ratio, fy=ratio)
img_orig = img

# convert to HLS
img =  cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

# take only the relevant channel (we probably want light)
channels = cv2.split(img)
img_channel = channels[channel]
# for Hue the ranges are 0-179. Saturate & Light are 0-255.
if channel == 0:
	max_value = 179
else:
	max_value = 255

# calc hist
rect_width = 40
rect_height = 40
overlap_x = 20
overlap_y = 20

#cv2.imshow('bah', img)
#while True:
#	if cv2.waitKey(1) == ord('q'):
#		break
'''
t = time.time()
for i in range(1):
	#hist = hist_rect(img_channel, 220, 330, 40, 40, max_value=max_value)
	hist = hist_rect(img_channel, 0, 0, None, None, max_value=max_value)
print 'hist:', time.time()-t

t = time.time()
for i in range(1000):
	x = is_black_white(hist)
print 'is_black_white:', time.time()-t
'''


for x, y in hist_iter_rects(img_channel, rect_width, rect_height, overlap_x, overlap_y):
	# draw a rectangle around found qrcodes
	print x, y
	pts = [[x, y], [x+rect_width, y], [x+rect_width, y+rect_height], [x, y+rect_width]]
	#pts = [[y, x], [y, x+rect_width], [y+rect_height, x+rect_width], [y+rect_width, x]]


	poly1 = np.array(pts, np.int32) #.reshape((-1,1,2))
	polys = [poly1]
	cv2.polylines(img_orig, polys, True, (0, 0, 255))


print 'time:', time.time()-t
cv2.imshow('bah', img_orig)
while True:
	if cv2.waitKey(1) == ord('q'):
		break
#hist = cv2.calcHist([img_channel], [0], None, dims, ranges)
#hist = hist_rect(img_channel, 0, 0, None, None, max_value=max_value)

# for IMG_0527.JPG, target is at 220, 330 -> 280, 390
#hist = hist_rect(img_channel, 220, 330, 60, 60, max_value=max_value)
#print hist
#print is_black_white(hist)

# print the histogram
#pylab.plot(hist)
#pylab.show()


# equalize histogram
# actually, this takes away a lot of information. so we won't do this.
'''channel = cv2.equalizeHist(channel)
hist = cv2.calcHist([channel], [0], None, [32], [0,256])

print hist
print hist[0], hist[-1], (1.0*hist[0]+hist[-1])/sum(hist)
pylab.plot(hist)
pylab.show()
'''

