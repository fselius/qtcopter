#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import sys,tty,termios
from qtcopter.msg import controller_msg
import curses


import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch


if __name__ == '__main__':
    try:

        rospy.init_node('keyboard_control', anonymous=True)
        pub = rospy.Publisher('/pid_input',controller_msg,queue_size=10)

        msgObject = controller_msg()
        msgObject.x = msgObject.y = msgObject.z = msgObject.t = 0
        while not rospy.is_shutdown():
            inkey = _Getch()
            k=inkey()
            print str(k)
            if k=='\x1b[A':
                    print "up pressed - Throttle +"
                    msgObject.z += 0.1
                    print "publishing to pid:"
                    print str(msgObject)
                    pub.publish(msgObject)
            elif k=='\x1b[B':
                    print "down pressed - Throttle -"
                    msgObject.z -= 0.1
                    print "publishing to pid:"
                    print str(msgObject)
                    pub.publish(msgObject)
            elif k=='\x1b[C':
                    print "right pressed - Roll +"
                    msgObject.x += 0.1
                    print "publishing to pid:"
                    print str(msgObject)
                    pub.publish(msgObject)
            elif k=='\x1b[D':
                    print "left pressed - Roll -"
                    msgObject.x -= 0.1
                    print "publishing to pid:"
                    print str(msgObject)
                    pub.publish(msgObject)

    except rospy.ROSInterruptException:
        pass