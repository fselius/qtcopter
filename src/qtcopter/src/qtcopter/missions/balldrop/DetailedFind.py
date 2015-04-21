from geometry_msgs.msg import Transform
from qtcopter.missions import MissionState
import rospy


class DetailedFind(MissionState):
    def __init__(self):
        MissionState.__init__(self,
                              input_keys=['roi'],
                              outcomes=['succeeded',
                                        'aborted',
                                        'failed'])
        self._pub = rospy.Publisher('/distance_to_target', Transform)

    def on_execute(self, userdata, image, height):
        '''
        Finds the target and publishes the transformation to it.

        If the target is lost, return 'failed'.
        If the optimal position is reached, return 'succeeded'.
        '''
        rospy.loginfo('Searching target and publishing Transform to it.')
        # TODO
        rospy.sleep(5)
        # self._pub.publish()
        # return None
        rospy.loginfo('Lost target, try finding it again.')
        return 'failed'
