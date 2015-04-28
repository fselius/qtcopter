from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image
from qtcopter.missions import MissionState
from qtcopter.missions.balldrop.polarity_find import find_target
import rospy
import cv2


class DetailedFind(MissionState):
    def __init__(self):
        MissionState.__init__(self,
                              input_keys=['roi'],
                              outcomes=['succeeded',
                                        'aborted',
                                        'failed'])
        self._t_pub = rospy.Publisher('/distance_to_target', Transform, queue_size=1)
        self._i_pub = rospy.Publisher('/detailed_find', Image, queue_size=1)

    def on_execute(self, userdata, image, height):
        '''
        Finds the target and publishes the transformation to it.

        If the target is lost, return 'failed'.
        If the optimal position is reached, return 'succeeded'.
        '''
        # Cut out ROI
        min, max = userdata.roi
        rospy.loginfo('Searching for target in ROI ({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min[0], min[1], max[0], max[1]))
        image = image[min[1]:max[1], min[0]:max[1]]

        # Find center of target
        center = find_target(image)

        if center is None:
            rospy.loginfo('Lost target, try finding it again.')
            return 'failed'

        if self._i_pub.get_num_connections() > 0:
            rospy.logdebug('Publishing center location of target.')
            cv2.circle(image, center, int(height*10), (0, 255, 0), int(height))
            img_msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
            self._i_pub.publish(img_msg)

        rospy.logdebug('Publishing transformation to target.')
        # TODO
        # self._t_pub.publish()
        return None
