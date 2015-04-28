from qtcopter.missions import MissionState
from qtcopter.missions.balldrop.hist_find import find_roi
import rospy
from sensor_msgs.msg import Image
import cv2


class CoarseFind(MissionState):
    def __init__(self):
        MissionState.__init__(self,
                              outcomes=['succeeded',
                                        'aborted'],
                              output_keys=['roi'])
        self._pub = rospy.Publisher('coarse_find', Image, queue_size=1)

    def on_execute(self, userdata, image, height):
        '''
        Runs the "coarse find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Trying to find coarse ROI in image.')
        roi = find_roi(image)
        if self._pub.get_num_connections() > 0:
            rospy.logdebug('Publishing coarse ROI.')
            print(roi)
            cv2.rectangle(image, roi[0], roi[1], (255, 0, 0))
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self._pub.publish(img_msg)
        userdata.roi = roi
        return 'succeeded'
