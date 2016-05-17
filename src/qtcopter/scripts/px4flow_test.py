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
import sys


class Flow:
    def __init__(self, flow_serial='/dev/ttyACM0'):
        self.flow_serial = flow_serial
        self.X = 0
        self.Y = 0
        self.p_time = []
        self.p_speed_x = []
        self.p_speed_y = []
        self.p_x = []
        self.p_y = []


        # TODO: save "last 10s" measurements, see how many are good (not zero, quality > ?)
        self.last = []
        self.total = 0
        self.zero = 0
        self.total_int_time = 0
    def callback(self, data):
        #global X, Y, last, total, zero, total_int_time

        #print data

        self.X += data.integrated_x*data.distance
        self.Y += data.integrated_y*data.distance
        self.p_x.append(self.X)
        self.p_y.append(self.Y)
        print 'X:', self.X, 'Y:', self.Y
        self.total += 1
        self.total_int_time += data.integration_time_us*1e6

        '''
        if data.integration_time_us != 0:
            speed_x = data.integrated_x/(1.0*data.integration_time_us/1e6)*data.distance
            speed_y = data.integrated_y/(1.0*data.integration_time_us/1e6)*data.distance
            #p_time.append(data.header.stamp.secs+1.0*data.header.stamp.nsecs/10**9)
            p_time.append(1.0*data.time_usec/1e6)
            p_speed_x.append(speed_x)
            p_speed_y.append(speed_y)
            #print 'quality: %03d range: %+.3f speed: %+.5f %+.5f location: %f %f' % (data.quality, data.distance, speed_x, speed_y, X, Y)
        else:
            zero += 1
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        '''

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'talker' node so that multiple talkers can
        # run simultaneously.
        print 'init node..'
        rospy.init_node('listener', anonymous=True)
        print 'subscribe..'
        rospy.Subscriber("/px4flow/px4flow/raw/optical_flow_rad", OpticalFlowRad, self.callback)

        print 'spin!'

        # spin() simply keeps python from exiting until this node is stopped
        t = time.time()
        rospy.spin()
        print 'run time:', time.time()-t
        print 'int time:', 1.*total_int_time / 1e6

    def listener_noros(self):
        import pymavlink.mavutil as mavutil
        m=mavutil.mavserial(self.flow_serial)
        # wait for a heartbeat, so we are synced with the protocol
        m.wait_heartbeat()
        #m.messages['OPTICAL_FLOW']
        last_print = 0
        self.sum_x = 0
        self.sum_y = 0
        while True:
            #x = m.recv_msg()
            x = m.recv_match(type='OPTICAL_FLOW_RAD', blocking=True)
            if x is None:
                print 'x is NONE!!'
                continue
            # TODO:
            #sum_x += x.flow_x
            self.callback(x)
            #print x
            if int(time.time()) > last_print:
                last_print = int(time.time())



if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            flow_serial = sys.argv[1]
        else:
            flow_serial = '/dev/ttyACM0'
        print 'Listening on', flow_serial
        f = Flow(flow_serial)
        f.listener_noros()
    except KeyboardInterrupt:
        pass
    if len(f.p_time) > 0:
        pl.plot(f.p_time, f.p_speed_x, 'go')
        pl.plot(f.p_time, f.p_speed_y, 'ro')
        pl.show()
    


