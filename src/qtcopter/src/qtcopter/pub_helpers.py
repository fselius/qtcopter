import rospy
import cv2


def draw_roi(image, rois):
    """
    Draw ROIs onto a copy of the current camera image
    and return it.
    """
    if rois is None:
        return None

    rospy.logdebug('Publishing coarse ROI.')
    image = image.copy()
    # cv2.drawContours(image, rois, -1, (0, 0, 255), 3)
    for roi in rois:
        cv2.rectangle(image, roi[0:2],
                      (roi[0]+roi[2], roi[1]+roi[3]),
                      (0, 255, 255), 3)
    return image
