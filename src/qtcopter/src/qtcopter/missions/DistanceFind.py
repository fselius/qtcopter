import numpy as np
import cv2


class DistanceFind:
    def __init__(self, lower, upper, min_factor=0.95, max_factor=1.05):
        assert(len(lower) == 3)
        assert(len(upper) == 3)
        self.lower = np.array(lower, dtype='uint8')
        self.upper = np.array(upper, dtype='uint8')
        self.min_factor = min_factor
        self.max_factor = max_factor

    def find_roi_mask(self, image):
        # Apply a small blur to reduce noise.
        image = cv2.blur(image, (15, 15))
        roi_mask = cv2.inRange(image, self.lower, self.upper)
        return roi_mask
    def find_roi_contours(self, image):
        roi_mask = self.find_roi_mask(image)
        img, contours, hier = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #self.debug_output(image, roi_mask)
        return contours
    def find_roi(self, image):
        roi_mask = self.find_roi_mask(image)
        y_indices, x_indices = np.where(roi_mask)
        #self.debug_output(image, roi_mask)
        
        if x_indices.size == 0 or y_indices.size == 0:
            return None

        y_min = int(self.min_factor*np.min(y_indices))
        x_min = int(self.min_factor*np.min(x_indices))
        y_max = int(self.max_factor*np.max(y_indices))
        x_max = int(self.max_factor*np.max(x_indices))

        return ((x_min, y_min), (x_max, y_max))

    def debug_output(self, image, roi_mask):
        # DEBUG OUTPUT
        output = cv2.bitwise_and(image, image, mask=roi_mask)
        cv2.imshow('Out', output)
        cv2.waitKey(5000)