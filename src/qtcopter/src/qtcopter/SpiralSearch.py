import rospy
from .pub_helpers import draw_roi, draw_rois
from math import sin, cos, pi


class SpiralSearch:
    """
    Fly spirals until we found the exact place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['continue spiraling', 'found target']

    def __init__(self, coarse_find_func, detailed_find_func,
                 spiral_time, spiral_factor):
        self.__coarse_find_func = coarse_find_func
        self.__detailed_find_func = detailed_find_func

        self.__spiral_factor = spiral_factor
        self.__spiral_time = spiral_time

    def __call__(self, userdata, output):
        rospy.loginfo('Finding coarse ROIs in image, height = {0}.'
                      .format(userdata.height_msg.range))
        rois = self.__coarse_find_func(userdata.image, height=userdata.height_msg.range,
                                       camera=userdata.camera)

        if rois is None or len(rois) == 0:
            # No ROIs found, spiral further.
            return self.publish_spiral(userdata, output)

        # Publish ROIs for debugging.
        userdata.publish_debug_image(draw_rois, rois)

        for roi in rois:
            roi = self.verify_roi(userdata, roi)
            if roi is not None:
                userdata.publish_debug_image(draw_roi, roi, color=(255, 0, 0))
                # Found target, stop!
                output.roi = roi
                userdata.publish_delta__keep_height(x=0, y=0, theta=0,
                                                    target_height=userdata.max_height)
                return 'found target'

        # No ROI contained the target, spiral further.
        return self.publish_spiral(userdata, output)

    def verify_roi(self, userdata, roi):
        min_ = (max(roi[0]-roi[2]/10, 0), max(roi[1]-roi[3]/10, 0))
        max_ = (roi[0]+roi[2]+roi[2]/10, roi[1]+roi[3]+roi[3]/10)

        cropped_image = userdata.image[min_[1]:max_[1], min_[0]:max_[0]]

        center, size = self.__detailed_find_func(cropped_image)
        if center is not None:
            rospy.loginfo('Found target in ROI '
                          '({0:d}, {1:d}), ({2:d}, {3:d}).'.format(min_[0],
                                                                   min_[1],
                                                                   max_[0],
                                                                   max_[1]))
            # Move ROI to center (to improve subsequent searches).
            roi_size = 1.5*size
            return (center[0]-roi_size/2, center[1]-roi_size/2, roi_size, roi_size)

        # Target not found in this ROI.
        return None

    def publish_spiral(self, userdata, output):
        if 't' in userdata:
            output.t = userdata.t
        else:
            output.t = rospy.Time.now()
        t = (rospy.Time.now() - output.t).to_sec()/self.__spiral_time
        a = self.__spiral_factor
        x = a*t*cos(t*2.0*pi)
        y = a*t*sin(t*2.0*pi)
        userdata.publish_delta__keep_height(x=x, y=y, theta=0,
                                            target_height=userdata.max_height)

        return 'continue spiraling'
