#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import time
from mavros.msg import OverrideRCIn
from mavros.srv import CommandBool

#Perform some tests with rc/override channels
class Mission:
    def __init__(self):
        rospy.init_node('preserve_height', anonymous=True)
        self.armDisarmService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.RcOverrideTopic = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.RcMessage = OverrideRCIn()

    def Arm(self):
        for i in range(0,7):
            self.RcMessage.channels[i]=OverrideRCIn.CHAN_NOCHANGE
        self.RcMessage.channels[2]=1000 #Set throttle to low speed so we can arm
        self.RcOverrideTopic.publish(self.RcMessage)
        time.sleep(1)
        self.armDisarmService(True)

    def Disarm(self):
        self.RcMessage = OverrideRCIn()
        for i in range(0,7):
            self.RcMessage.channels[i]=OverrideRCIn.CHAN_NOCHANGE
        self.RcMessage.channels[2]=1000 #Set throttle to low speed so we can arm
        self.RcOverrideTopic.publish(self.RcMessage)
        time.sleep(1)
        self.armDisarmService(0)

    #Set value to specific rc channel
    def SetRcMessage(self, channel, value):
        channel = int(channel)
        value = int(value)
        for i in range(0,7):
            if channel == i:
                self.RcMessage.channels[i]=value
            else:
                self.RcMessage.channels[i]=OverrideRCIn.CHAN_NOCHANGE

    def TestMission(self):
        self.Arm()
        time.sleep(2)
        self.SetRcMessage(2,1150)
        self.RcOverrideTopic.publish(self.RcMessage)
        print("Setting throttle to 1150 for 5 seconds")
        time.sleep(5)

        self.SetRcMessage(1,1800)
        self.RcOverrideTopic.publish(self.RcMessage)
        print("Setting pitch to 1600 for 5 seconds")
        time.sleep(5)

        self.SetRcMessage(2,1200)
        self.RcOverrideTopic.publish(self.RcMessage)
        print("Setting throttle to 1250 for 5 seconds")
        time.sleep(2)

        self.SetRcMessage(0,1200)
        self.RcOverrideTopic.publish(self.RcMessage)
        print("Setting Roll to 1700 for 5 seconds")
        time.sleep(5)

        self.SetRcMessage(2,1100)
        self.RcOverrideTopic.publish(self.RcMessage)
        print("Setting throttle to 1100 for 5 seconds")
        time.sleep(2)

        self.Disarm()


if __name__ == '__main__':
    try:
        tm = Mission()
        print("Initated Mission Class")
        print("Performing Test Mission")
        tm.TestMission()
        print("Test Mission Done")
    except rospy.ROSInterruptException:
        pass