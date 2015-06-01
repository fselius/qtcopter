import numpy as np

def rect_contour(rect):
    " create contour from rectangle "
    # rect = (x, y, width, height)
    pts = [
        [[rect[0], rect[1]]],
        [[rect[0]+rect[2]-1, rect[1]]],
        [[rect[0]+rect[2]-1, rect[1]+rect[3]-1]],
        [[rect[0], rect[1]+rect[3]-1]]
    ]
    return np.array(pts, dtype=np.int32)


