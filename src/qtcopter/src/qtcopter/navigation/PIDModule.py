#!/usr/bin/env python
# Software License Agreement (BSD License)
import rospy

class PIDControl:
    def __init__(self):
        rospy.init_node('PIDControl', anonymous=True)
        self.ControlOutTopic = rospy.Publisher('PID/Out')

        rospy.Subscriber('/mavros/global_position/rel_alt', Float64, self.callback)
        overridePub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rate = rospy.Rate(1) # 10hz
        self.Arm()
        self.init_channels()



if __name__ == '__main__':
    try:
        Controllers = PIDControl()
    except rospy.ROSInterruptException:
        pass