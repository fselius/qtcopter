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
import threading


class Flow:
    def __init__(self, flow_serial='/dev/ttyACM0', dummy=False):
        self.flow_serial = flow_serial
        self.X = 0
        self.Y = 0
        self.p_time = []
        self.p_speed_x = []
        self.p_speed_y = []
        self.p_x = []
        self.p_y = []

        self.dummy = dummy


        # TODO: save "last 10s" measurements, see how many are good (not zero, quality > ?)
        self.last = []
        self.total = 0
        self.zero = 0
        self.total_int_time = 0

        self.thread = None
        self.thread_please_die = False
    def __del__(self):
        self.stop()

    def lock(self):
        # TODO: use this to lock self.X, self.Y -- atomically set them
        pass
    def unlock(self):
        # TODO: use this to unlock self.X, self.Y -- atomically set them
        pass
    def set_xy(self, x, y):
        self.lock()
        self.X = x
        self.Y = y
        self.unlock()
    def callback(self, data):
        #global X, Y, last, total, zero, total_int_time

        #print data
        self.lock()
        self.X += data.integrated_x*data.distance
        self.Y += data.integrated_y*data.distance
        x = self.X
        y = self.Y
        self.unlock()

        self.p_x.append(x)
        self.p_y.append(y)
        print 'flow callback: X:', x, 'Y:', y
        self.total += 1
        self.total_int_time += data.integration_time_us*1e6


    def run(self):
        if self.ros:
            listener = self.listener
        elif self.dummy:
            listener = self.listener_dummy
        else:
            listener = self.listener_noros

        # TODO: spawn thread
        self.thread = threading.Thread(target=listener)
        self.thread.run()

    def stop(self):
        self.thread_please_die = True
        if self.thread and self.thread.isAlive():
            self.thread.join(1)
            if self.thread.isAlive():
                print >> sys.stderr, "Flow(): thread could not be joined"

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
    


