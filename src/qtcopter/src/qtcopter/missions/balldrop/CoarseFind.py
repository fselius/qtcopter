from qtcopter.missions import MissionState
import rospy


class CoarseFind(MissionState):
    def __init__(self):
        MissionState.__init__(self,
                              outcomes=['succeeded',
                                        'aborted'],
                              output_keys=['roi'])

    def on_execute(self, userdata, image, height):
        '''
        Runs the "coarse find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Trying to find coarse ROI in image.')
        # TODO
        rospy.sleep(5)
        userdata.roi = ((0, 0), image.shape)
        return 'succeeded'
