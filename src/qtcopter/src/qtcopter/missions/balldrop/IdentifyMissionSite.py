#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image

class IdentifyMissionSite:
    def __init__(self):
        pass

    def identify_possible_targets(self, image):
        """ Identify pixel positions of possible targets """
        rospy.logdebug('Identify pixel positions of possible targets.')
        # Idea: convert to HSL, threshold on lightness
        pass

    def get_direction_to_target(self, image):
        """ Find possible targets and return movement instructions towards it. """

if __name__ == '__main__':
    identify = IdentifyMissionSite()
    def image_callback(image):
        identify(image)
    rospy.init_node('balldrop_identify')
    rospy.Subscriber('image_in', Image, image_callback)
    rospy.spin()
