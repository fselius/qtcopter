from geometry_msgs.msg import Transform
from qtcopter.missions import MissionState
from qtcopter.missions.balldrop.coords import cam_pixel_to_xy
import rospy
import cv2


class DetailedFind(MissionState):
    def __init__(self, debug_pub, find_object_center_func):
        MissionState.__init__(self,
                              debug_pub,
                              input_keys=['roi'],
                              outcomes=['succeeded',
                                        'aborted',
                                        'failed'])
        self._find_object_center = find_object_center_func
        self._pub = rospy.Publisher('/distance_to_target', Transform,
                                    queue_size=1)

    def on_execute(self, userdata, image, height):
        '''
        Finds the target object and publishes the transformation to it.

        If the target is lost, return 'failed'.
        If the optimal position is reached, return 'succeeded'.
        '''
        # Cut out ROI
        min, max = userdata.roi
        rospy.loginfo('Searching for target in ROI '
                      '({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min[0],
                                                               min[1],
                                                               max[0],
                                                               max[1]))
        cropped_image = image[min[1]:max[1], min[0]:max[0]]

        # Find center of target
        center = self._find_object_center(cropped_image)
        if center is not None:
            # Add offset to uncropped image
            center = (center[0] + min[0], center[1] + min[1])

        def draw_center_location():
            rospy.logdebug('Publishing center location of target.')
            cv2.rectangle(image, userdata.roi[0], userdata.roi[1], (255, 0, 0))
            if center is not None:
                cv2.circle(image, center, int(height*10),
                           (0, 255, 0), -1)
            return image
        self.debug_publish(draw_center_location)

        if center is None:
            rospy.loginfo('Lost target, try finding it again.')
            return 'failed'

        rospy.logdebug('Publishing transformation to target.')
        # TODO - determine camera to ball offset, save it somewhere
        CAM_BALL_OFFSET_X, CAM_BALL_OFFSET_Y = (0, 0)
        x, y = cam_pixel_to_xy(height, image.shape[1], image.shape[0], center[0], center[1], CAM_BALL_OFFSET_X, CAM_BALL_OFFSET_Y)
        # TODO: save x, y, height offset to output
        #self._pub.publish()
        #return 'succeeded'
        return None
