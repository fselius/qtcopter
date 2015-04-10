#!/usr/bin/env python
# Software License Agreement (BSD License)

import time
import rospy
from mavros.msg import OverrideRCIn
from mavros.srv import CommandBool
from std_msgs.msg import Float64
from mavros.srv import SetMode

#Hover with the drone at certain height for few seconds
#Don't forget to
class PreserveHeight:

    def __init__(self, height, alt_hold_time):
        self.overridePub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.armDisarmService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.height = height
        self.reached = False
        self.alt_hold_time = alt_hold_time
        self.msg = OverrideRCIn()
        dir(self.msg)

    def Arm(self):
        for i in range(0,7):
            self.msg.channels[i]=OverrideRCIn.CHAN_NOCHANGE
        self.msg.channels[2]=1000 #Set throttle to low speed so we can arm
        self.overridePub.publish(self.msg)
        time.sleep(1)
        self.armDisarmService(True)

    #set all rc_channels with mid value, set throttle to elevation speed
    def init_channels(self):
        for i in range(0, 7):
            self.msg.channels[i] = 1500
        self.msg.channels[2] = 1400

    #Callback method for the rel_altitude topic
    def callback(self, data):
        if data.data > self.height and not self.reached:
            self.reached = True
            try:
                set_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
                set_mode(base_mode=0, custom_mode='ALT_HOLD')
                print "Holding Altitude for %d seconds"%self.alt_hold_time
                time.sleep(self.alt_hold_time)
                set_mode(base_mode=0, custom_mode='LAND')
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    #All init tasks for this node will be perfomed here
    def initialize(self):
        rospy.init_node('preserve_height', anonymous=True)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.callback)
        overridePub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rate = rospy.Rate(1) # 10hz
        self.Arm()
        self.init_channels()
        while not rospy.is_shutdown():
            overridePub.publish(self.msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        preserve = PreserveHeight(2,20) #Keep at 2 meters, 20 seconds
        preserve.initialize()
    except rospy.ROSInterruptException:
        pass