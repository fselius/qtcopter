import cv2


def draw_rois(image, rois, color=(0, 255, 255)):
    """
    Draw ROIs onto a copy of the given image and return it.
    """
    if rois is None:
        return None

    image = image.copy()
    # cv2.drawContours(image, rois, -1, (0, 0, 255), 3)
    for roi in rois:
        cv2.rectangle(image, (int(roi[0]), int(roi[1])),
                      (int(roi[0]+roi[2]), int(roi[1]+roi[3])),
                      color, 3)
    return image


def draw_roi(image, roi, color=(0, 255, 255)):
    """
    Draw ROI onto a copy of the given image and return it.
    """
    if roi is None:
        return None

    image = image.copy()
    cv2.rectangle(image, (int(roi[0]), int(roi[1])),
                  (int(roi[0]+roi[2]), int(roi[1]+roi[3])),
                  color, 3)
    return image


def draw_circle(image, center, diameter, circle_color=(0, 255, 0),
                center_color=(0, 255, 0)):
    """
    Draw circle onto a copy of the given image and return it.
    """

    if center is None or diameter is None:
        return None

    image = image.copy()
    # Plot circle.
    cv2.circle(image, (int(center[0]), int(center[1])), int(diameter/2.0),
               circle_color, 4)
    # Plot center.
    cv2.circle(image, (int(center[0]), int(center[1])), 7,
               center_color, -1)
    return image
