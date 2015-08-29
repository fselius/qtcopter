#!/usr/bin/env python

import fileinput
#import pylab as pl
#import time
#import matplotlib.pyplot as plt
#import numpy as np
#import numpy.fft as fft
import numpy
def median(lst):
    return numpy.median(numpy.array(lst))

lpf = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
previous_height = 2

for line in fileinput.input():
    if 'range' in line:
        distance = float(line.split()[1])
        if distance == 0:
            distance = previous_height

        lpf_current_distance = distance
        if median(lpf) - 0.3 > distance or median(lpf) + 0.3 < distance:
            distance = previous_height

        #for i in xrange(len(lpf)):
        #    if lpf[len(lpf) - 1 - i] + 0.020 * (i + 1) < distance or lpf[len(lpf) - 1 - i] - 0.020 * (i + 1) > distance:
        #        distance = previous_height
            
        previous_height = distance
        lpf.append(lpf_current_distance)
        lpf.pop(0)
        print lpf_current_distance,distance
