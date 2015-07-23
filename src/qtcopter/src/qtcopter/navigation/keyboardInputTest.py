#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from qtcopter.msg import controller_msg
from getch import getch, pause


if __name__ == '__main__':
    try:
        #supported keys:
        #(a,d) -> Control ROLL
        #(w,x) -> control PITCH
        #(q,e) -> control THROTTLE
        #(s) -> reset all channels (roll,pitch,throttle -> 0)
        #(0) -> exit

        rospy.init_node('keyboard_control', anonymous=True)
        pub = rospy.Publisher('/pid_input',controller_msg,queue_size=10)

        msgObject = controller_msg()
        msgObject.x = msgObject.y = msgObject.z = msgObject.t = 0
        while not rospy.is_shutdown():
            k = getch.getch()
            if k=="e":
                    print "up pressed - Throttle +"
                    msgObject.z += 0.1
            elif k=='q':
                    print "down pressed - Throttle -"
                    msgObject.z -= 0.1

            elif k=='d':
                    print "right pressed - Roll +"
                    msgObject.y += 0.1
            elif k=='a':
                    print "left pressed - Roll -"
                    msgObject.y -= 0.1
            elif k=='w':
                    print "left pressed - Pitch +"
                    msgObject.x += 0.1
            elif k=='x':
                    print "left pressed - Pitch -"
                    msgObject.x -= 0.1
            elif k=='s':
                    print "s pressed - Reset All"
                    msgObject.x = msgObject.y = msgObject.z = 0
            elif k=='0':
                break

            print "publishing to pid:"
            print str(msgObject)
            pub.publish(msgObject)

    except rospy.ROSInterruptException:
        pass