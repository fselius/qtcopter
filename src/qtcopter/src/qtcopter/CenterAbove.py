import rospy
import cv2
from .pub_helpers import draw_rois


class CenterAbove:
    """
    Fly exactly above the place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['target lost', 'centered above target', 'moving above target']

    def __init__(self, find_func):
        self.__find_func = find_func

    def __call__(self, userdata, output):
        """
        Runs the coarse find algorithm and returns a region of interest.
        """
        rospy.loginfo('Trying to find coarse ROI in image, height = {0}.'
                      .format(userdata.height_msg.range))

        # Publish ROIs for debugging.
        userdata.publish_debug_image(draw_rois, roi)
        output.rois = rois

        # Choose first ROI (should be sorted)
        roi = rois[0]
        image_width = userdata.image.shape[1]
        image_height = userdata.image.shape[0]

        # TODO: find ROI center. Currently simply center of bounding rectangle,
        # but we can use center of mass
        center = (roi[0]+roi[2]/2.0, roi[1]+roi[3]/2.0)

        # Get distance in meters
        offset = userdata.camera.get_ground_offset(center, userdata.height_msg.range)

        # if target near edge, move to edge
        def is_edge_close(value, size):
            return 1.0*value/size < 0.1 or 1.0*(size-value)/size < 0.1

        close_left = is_edge_close(roi[0], image_width)
        close_top = is_edge_close(roi[1], image_height)
        close_right = is_edge_close(roi[0]+roi[2], image_width)
        close_bot = is_edge_close(roi[1]+roi[3], image_height)

        # If ROI is not close to edge..
        if not (close_left or close_top or close_right or close_bot):
            return 'centered above target'

        # If center of mass is nearby, don't move
        if abs(offset[0]) < 0.05 and abs(offset[1]) < 0.05:
            return 'centered above target'

        # finally, fly to center of mass
        userdata.publish_delta__keep_height(x=offset[0], y=offset[1], theta=0,
                                            target_height=userdata.max_height)

        # continue with coarse find until we are above ROI
        return 'moving above target'
