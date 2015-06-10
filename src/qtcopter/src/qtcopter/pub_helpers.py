import cv2


def draw_rois(image, rois, color=(0, 255, 255)):
    """
    Draw ROIs onto a copy of the current camera image
    and return it.
    """
    if rois is None:
        return None

    image = image.copy()
    # cv2.drawContours(image, rois, -1, (0, 0, 255), 3)
    for roi in rois:
        cv2.rectangle(image, roi[0:2],
                      (roi[0]+roi[2], roi[1]+roi[3]),
                      color, 3)
    return image


def draw_roi(image, roi, color=(0, 255, 255)):
    """
    Draw ROI onto a copy of the current camera image
    and return it.
    """
    if roi is None:
        return None

    image = image.copy()
    cv2.rectangle(image, roi[0:2],
                  (roi[0]+roi[2], roi[1]+roi[3]),
                  color, 3)
    return image
