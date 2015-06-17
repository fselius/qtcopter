from . import MissionState
import rospy
import cv2

class DetailFind(MissionState):
    def __init__(self, debug_pub, find_object):
        MissionState.__init__(self,
                              debug_pub,
                              input_keys=['rois'],
                              outcomes=['succeeded',
                                        'aborted',
                                        'failed'])
        self._find_object = find_object

    def on_execute(self, userdata, image, height):
        '''
        Finds the target object and publishes the transformation to it.

        If the target is lost, return 'failed'.
        If the optimal position is reached, return 'succeeded'.
        '''
        debug_draw_rects = []
        debug_draw_circles = []

        return_value = 'failed'
        for cont in userdata.rois:
            rect = cv2.boundingRect(cont)
            # Cut out ROI
            min, max = (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3])
            debug_draw_rects.append((min, max))
            rospy.loginfo('Searching for target in ROI '
                          '({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min[0],
                                                                   min[1],
                                                                   max[0],
                                                                   max[1]))
            cropped_image = image[min[1]:max[1], min[0]:max[0]]

            # Find center of target
            data = self._find_object(cropped_image)
            if data is not None:
                print 'find_object ->', data
                #center = (int(center[0] + min[0]), int(center[1] + min[1]))
                #debug_draw_circles.append((center, size))
                #rospy.logdebug('Publishing offset to target.')
                #self.publish_offset(center, size)
                # TODO: when mission finished:
                # return_value = 'succeeded'
                #return_value = None
                break

        def draw_center_location():
            rospy.logdebug('Publishing center location of target.')
            for min, max in debug_draw_rects:
                cv2.rectangle(image, min, max, (255, 0, 0))
            for center, size in debug_draw_circles:
                cv2.circle(image, center, int(size/2.0),
                           (0, 255, 0), -1)
            return image
        #self.debug_publish(draw_center_location)


        rospy.loginfo('Lost target, try finding it again.')
        return return_value

