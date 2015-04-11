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
	total = sum(hist)
	# find low edge
	cur = 0
	for i in range(len(hist)):
		cur += hist[i]
		if 1.0*cur/total > cut:
			# this cell adds up to more than cut, so we don't want to cut it.
			break
	i -= 1
	low = max(0, i) # if i < 0..

	# find high edge
	cur = 0
	for i in range(len(hist)-1, -1, -1):
		cur += hist[i]
		if 1.0*cur/total > cut:
			break
	i += 1
	high = min(len(hist)-1, i) # if i >= len(hist)

	return low, high


	
def is_black_white(hist):
	" decide whether histogram is of black/white pattern "
	low, high = find_edges(hist)	
	#print 'low, high, total hist length:', low, high, len(hist)

	hist = hist[low:high+1]

	# take lower and upper % buckets (darkest + lightest), if they
	# contain more than threshold, good
	BUCKETS = 0.05 # precentage of lower and upper
	THRESHOLD = 0.5 # how much these buckets should have

	buckets = int(BUCKETS*len(hist))
	# TODO: in case BUCKETS*len(hist) is not a whole, should we take part
	# of the next bucket as well?
	sides_sum = sum(hist[:buckets]) + sum(hist[-buckets:])
	print 'low:', 1.0*sum(hist[:buckets])/sum(hist), 'high:', 1.0*sum(hist[-buckets:])/sum(hist), 'total:', 1.0*sides_sum/sum(hist)
	if 1.0*sides_sum/sum(hist) > THRESHOLD:
		return True
	return False

#def 

# open image
img = cv2.imread(img_path)
t = time.time()

# convert to HLS
img =  cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

# resize to have maximum 500px width/height
ratio = 500./max(img.shape[:2])
img = cv2.resize(img, (0,0), fx=ratio, fy=ratio)

# take only the relevant channel (we probably want light)
channels = cv2.split(img)
img_channel = channels[channel]
# for Hue the ranges are 0-179. Saturate & Light are 0-255.
if channel == 0:
	ranges = [0, 180]
	dims = [180]
else:
	ranges = [0, 256]
	dims = [256]

# calc hist
hist = cv2.calcHist([img_channel], [0], None, dims, ranges)
#print hist
print is_black_white(hist)

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

print 'time:', time.time()-t
