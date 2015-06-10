from . import MissionState
import rospy
import cv2
from ..navigation.Camera import default_camera as camera
from .utils import rect_contour

class DetailedFind(MissionState):
    def __init__(self, delta_pub, debug_pub, find_object_center_func):
        MissionState.__init__(self,
                              delta_pub,
                              debug_pub,
                              input_keys=['rois'],
                              output_keys=['rois'],
                              outcomes=['succeeded',
                                        'aborted',
                                        'failed'])
        self._find_object_center = find_object_center_func

    def publish_offset(self, center, estimated_size):
        # Get size of target in millimeters
        real_size = rospy.get_param('target/size')/1000.0  # meters!

        # Get offset in pixels
        offset = [center[0]-camera.width/2.0, center[1]-camera.height/2.0]
        # Get distance in millimetres

        # find size at 1meter
        p1 = (center[0]-estimated_size/2.0, center[1])
        p2 = (center[0]+estimated_size/2.0, center[1])
        p1 = camera.get_ground_offset(p1, 1)
        p2 = camera.get_ground_offset(p2, 1)
        size_at_1m = cv2.norm(p1, p2)
        distance = real_size/size_at_1m  # estimated :)

        offset = camera.get_ground_offset(center, distance)
        # TODO: f from camera calibration
        #f = 28
        #distance = f*real_size/estimated_size
        #offset[0] = f*offset[0]/distance
        #offset[1] = f*offset[1]/distance

        if rospy.get_param('camera/pointing_downwards'):
            # Camera points to the ground
            # -> distance to object is z coordinate
            x, y = offset
            z = distance
            camera_frame = 'downward_cam_optical_frame'
        else:
            # Camera is facing forward
            # -> distance to object is y coordinate
            x, z = offset
            y = distance
            camera_frame = 'cam_optical_frame'

        self.publish_delta(camera_frame, (x, y, z), 0)

    def on_execute(self, userdata, image, height):
        '''
        Finds the target object and publishes the transformation to it.

        If the target is lost, return 'failed'.
        If the optimal position is reached, return 'succeeded'.
        '''
        debug_draw_rects = []
        debug_draw_circles = []
        new_rois = None

        return_value = 'failed'
        for cont in userdata.rois:
            rect = cv2.boundingRect(cont)
            # Cut out ROI
            #min, max = (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3])
            min_ = (max(rect[0]-rect[2]/10, 0), max(rect[1]-rect[3]/10, 0))
            max_ = (rect[0]+rect[2]+rect[2]/10, rect[1]+rect[3]+rect[3]/10)

            debug_draw_rects.append((min_, max_))
            rospy.loginfo('Searching for target in ROI '
                          '({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min_[0],
                                                                   min_[1],
                                                                   max_[0],
                                                                   max_[1]))
            cropped_image = image[min_[1]:max_[1], min_[0]:max_[0]]

            # Find center of target
            center, size = self._find_object_center(cropped_image)
            if center is not None:
                rospy.loginfo('Found target in ROI '
                              '({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min_[0],
                                                                       min_[1],
                                                                       max_[0],
                                                                       max_[1]))
                # Add offset to uncropped image
                center = (int(center[0] + min_[0]), int(center[1] + min_[1]))
                # move ROI to center (to improve subsequent searches)
                roi_size = 1.5*size
                new_rois = [rect_contour((center[0]-roi_size/2, center[1]-roi_size/2, roi_size, roi_size))]
                debug_draw_circles.append((center, size))
                rospy.logdebug('Publishing offset to target.')
                self.publish_offset(center, size)
                # TODO: when mission finished:
                # return_value = 'succeeded'
                return_value = None
                break

        if new_rois is not None:
            userdata.rois = new_rois

        def draw_center_location():
            rospy.logdebug('Publishing center location of target.')
            for min_, max_ in debug_draw_rects:
                cv2.rectangle(image, min_, max_, (255, 0, 0))
            for center, size in debug_draw_circles:
                # plot circle
                cv2.circle(image, center, int(size/2.0),
                           (0, 255, 0), 4)
                # plot middle
                cv2.circle(image, center, 7,
                           (0, 255, 0), -1)
            return image

        self.publish_debug_image(draw_center_location)

        if return_value is 'failed':
            rospy.loginfo('Lost target, try finding it again.')
        return return_value
