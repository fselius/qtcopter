from qtcopter.missions import MissionState
import rospy
import cv2


class CoarseFind(MissionState):
    def __init__(self, debug_pub, find_roi_func):
        MissionState.__init__(self,
                              debug_pub,
                              outcomes=['succeeded',
                                        'aborted'],
                              output_keys=['roi'])
        if find_roi_func is None:
            rospy.logwarn('CoarseFind: Using dummy ROI finder function.')
            self._find_roi = lambda image: ((0, 0), (image.shape[1], image.shape[0]))
        else:
            self._find_roi = find_roi_func

    def on_execute(self, userdata, image, height):
        '''
        Runs the "coarse find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Trying to find coarse ROI in image.')
        roi = self._find_roi(image)

        def draw_roi():
            rospy.logdebug('Publishing coarse ROI.')
            if roi is not None:
                cv2.rectangle(image, roi[0], roi[1], (255, 0, 0))
            return image
        self.debug_publish(draw_roi)

        if roi is None:
            return None

        userdata.roi = roi
        return 'succeeded'
