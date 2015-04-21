import rospy
import smach
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Range
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading
import numpy as np


class CoarseFind(smach.State):
    '''
    Based on smach.MissionState.
    '''
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted'],
                             output_keys=['roi'])
        self._trigger_event = threading.Event()
        self._bridge = CvBridge()
        rospy.loginfo('CoarseFind initialized.')

    def coarse_find(self, image, height):
        '''
        Runs the "coarse find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Trying to find coarse ROI in image.')
        # TODO
        rospy.sleep(5)
        return np.zeros(image.shape)

    def callback(self, range_msg, img_msg, userdata):
        height = range_msg.range
        try:
            image = self._bridge.imgmsg_to_cv2(img_msg,
                                               desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logerror(error)

        roi = self.coarse_find(image, height)

        if roi is None:
            rospy.loginfo('No candidate found, trying again.')
            return

        rospy.loginfo('Found candidate, continue.')
        userdata.roi = roi

        self._height_sub.sub.unregister()
        self._height_sub.sub.unregister()
        self._trigger_event.set()

    def execute(self, userdata):
        # If prempted before even getting a chance, give up.
        if self.preempt_requested():
            self.service_preempt()
            return 'aborted'

        self._trigger_event.clear()

        self._height_sub = Subscriber('/height', Range)
        self._image_sub = Subscriber('/image', Image)
        sync = ApproximateTimeSynchronizer([self._height_sub, self._image_sub],
                                           queue_size=1, slop=0.05)
        sync.registerCallback(self.callback, userdata)

        # Wait until a candidate has been found.
        self._trigger_event.wait()

        del sync

        if self.preempt_requested():
            self.service_preempt()
            return 'aborted'

        return 'succeeded'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
