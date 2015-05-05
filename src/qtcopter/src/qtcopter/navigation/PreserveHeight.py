#!/usr/bin/env python
# Software License Agreement (BSD License)

import time
import rospy
from std_msgs.msg import Float64
from qtcopter.msg import controller_msg
from Navigator import Navigator
from RcMessage import RcMessage

class PreserveHeight:

    def __init__(self, height):
        self.pidInput = rospy.Publisher('pid_input', controller_msg, queue_size=10)
        self.height = height
        rospy.Subscriber('/pid/controller_command',controller_msg,self.HandlePID)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.HandleAltitude)
        self.myNavigator = Navigator()


    def StartMission(self):
        print("Beginning Preserve Height Mission")
        print("Arming")
        if self.myNavigator.Arm(True):
            print("Armed")
        time.sleep(1)

    def HandlePID(self, data):
        self.myNavigator.PublishRCMessage(1500,1500,data.z,1500)
        print("PID Fix %s" % data.z)


    #Callback method for the rel_altitude topic
    def HandleAltitude(self, data):
        msg = controller_msg()
        msg.z = self.height - data.data #only working on alt values for now
        self.pidInput.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('preserve_height', anonymous=True)
        preserve = PreserveHeight(200) #Preserve Height at 200 meters
        preserve.StartMission()
        while not rospy.is_shutdown():
            time.sleep(0)
    except rospy.ROSInterruptException:
        pass