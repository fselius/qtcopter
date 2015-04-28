import numpy as np


class DiffFind:
    def __init__(self, color):
        assert(len(color) == 3)
        self.color = np.array(color)

    def find_roi(self, image):
        # Calculate difference image from color
        pass
