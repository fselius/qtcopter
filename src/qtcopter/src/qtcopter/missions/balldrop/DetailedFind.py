import rospy
import smach
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Range
from message_filters import ApproximateTimeSynchronizer, Subscriber
import threading


class DetailedFind(smach.State):
    '''
    Based on smach.MissionState.
    '''
    def __init__(self):
        smach.State.__init__(self,
                             input_keys=['roi'],
                             outcomes=['succeeded',
                                       'aborted',
                                       'failed'])
        self._trigger_event = threading.Event()
        self._bridge = CvBridge()
        rospy.loginfo('DetailedFind initialized.')

    def detailed_find(self, image, height):
        '''
        Runs the "detailed find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Searching target in image and printing distance to it.')
        # TODO
        rospy.sleep(5)
        # Return False if target has been lost
        return False

    def callback(self, range_msg, img_msg, userdata):
        height = range_msg.range
        try:
            image = self._bridge.imgmsg_to_cv2(img_msg,
                                               desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logerror(error)

        if not self.detailed_find(image, height):
            # Target lost, return to previous state
            self._height_sub.sub.unregister()
            self._image_sub.sub.unregister()
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

        # Unregister from topics and continue/abort.
        del sync

        if self.preempt_requested():
            self.service_preempt()
            return 'aborted'

        rospy.loginfo('Lost target, go back.')
        return 'failed'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
