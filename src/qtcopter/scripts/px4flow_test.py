#!/usr/bin/env python

"""
first run:
roslaunch mavros_extras px4flow.launch
"""

'''
import rospy
from std_msgs.msg import String
from mavros_extras.msg import OpticalFlowRad
'''

import pylab as pl
import time

X = 0
Y = 0
p_time = []
p_speed_x = []
p_speed_y = []
p_x = []
p_y = []


# TODO: save "last 10s" measurements, see how many are good (not zero, quality > ?)
last = []
total = 0
zero = 0
total_int_time = 0
def callback(data):
    global X, Y, last, total, zero, total_int_time

    print data

    X += data.integrated_x*data.distance
    Y += data.integrated_y*data.distance
    p_x.append(X)
    p_y.append(Y)
    #print 'X:', X, 'Y:', Y
    total += 1
    total_int_time += data.integration_time_us*1e6

    if data.integration_time_us != 0:
        speed_x = data.integrated_x/(1.0*data.integration_time_us/1e6)*data.distance
        speed_y = data.integrated_y/(1.0*data.integration_time_us/1e6)*data.distance
        #p_time.append(data.header.stamp.secs+1.0*data.header.stamp.nsecs/10**9)
        p_time.append(1.0*data.time_usec/1e6)
        p_speed_x.append(speed_x)
        p_speed_y.append(speed_y)
        print 'quality: %03d range: %+.3f speed: %+.5f %+.5f location: %f %f' % (data.quality, data.distance, speed_x, speed_y, X, Y)
    else:
        zero += 1
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    print 'init node..'
    rospy.init_node('listener', anonymous=True)
    print 'subscribe..'
    rospy.Subscriber("/px4flow/px4flow/raw/optical_flow_rad", OpticalFlowRad, callback)

    print 'spin!'

    # spin() simply keeps python from exiting until this node is stopped
    t = time.time()
    rospy.spin()
    print 'run time:', time.time()-t
    print 'int time:', 1.*total_int_time / 1e6

def listener_noros():
    import pymavlink.mavutil as mavutil
    m=mavutil.mavserial('/dev/ttyACM0')
    # wait for a heartbeat, so we are synced with the protocol
    m.wait_heartbeat()
    #m.messages['OPTICAL_FLOW']
    last_print = 0
    sum_x = 0
    sum_y = 0
    while True:
        #x = m.recv_msg()
        x = m.recv_match(type='OPTICAL_FLOW_RAD', blocking=True)
        if x is None:
            print 'x is NONE!!'
            continue
        # TODO:
        #sum_x += x.flow_x
        callback(x)
        #print x
        if int(time.time()) > last_print:
            last_print = int(time.time())



if __name__ == '__main__':
    try:
        listener_noros()
    except KeyboardInterrupt:
        pass
    if len(p_time) > 0:
        pl.plot(p_time, p_speed_x, 'go')
        pl.plot(p_time, p_speed_y, 'ro')
        pl.show()
    


