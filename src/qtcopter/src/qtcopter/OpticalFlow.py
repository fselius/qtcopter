import cv2
import numpy as np


class OpticalFlow:
    def __init__(self):
        self.__previous = None

    def get_movement(self, image):
        """
        Return movement in x and y direction from previous image.
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.__previous is None:
            movement = (None, None)
        else:
            flow = cv2.calcOpticalFlowFarneback(self.__previous, image,
                                                pyr_scale=0.5,
                                                levels=3,
                                                winsize=15,
                                                iterations=3,
                                                poly_n=5,
                                                poly_sigma=1.2,
                                                flags=0)
            movement = np.median(np.median(flow, axis=0), axis=0)

        self.__previous = image
        return movement

    @classmethod
    def from_ros(cls):
        return cls()
