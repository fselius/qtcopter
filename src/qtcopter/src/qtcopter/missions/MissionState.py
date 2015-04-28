from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, Range
import rospy
import smach
import threading
import cv2


class MissionState(smach.State):
    def __init__(self, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(self,
                             outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self._trigger_event = threading.Event()
        self._bridge = CvBridge()
        self._input_keys = input_keys
        self._output_keys = output_keys
        rospy.logdebug('%s initialized.'.format(self))

    def on_execute(self):
        '''
        Returns the successor state (one of the "outcomes") or
        None if state should continue to run.
        '''
        return NotImplemented

    def callback(self, range_msg, img_msg, userdata, finishdata):
        if self._finished:
            return

        height = range_msg.range
        try:
            image = self._bridge.imgmsg_to_cv2(img_msg,
                                               desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logerror(error)

        # Apply a small blur to reduce noise.
        image = cv2.GaussianBlur(image, (3, 3), 0, 0)
        output = self.on_execute(userdata, image, height)

        if output is None:
            # Don't terminate this state yet,
            # try again with new sensor input.
            return

        self._finished = True

        finishdata.append(output)

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

        finishdata = []
        self._finished = False
        sync.registerCallback(self.callback, userdata, finishdata)

        # Wait until a candidate has been found.
        self._trigger_event.wait()
        del sync

        if self.preempt_requested():
            self.service_preempt()
            return 'aborted'

        assert(len(finishdata) == 1)
        return finishdata[0]

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.set()
