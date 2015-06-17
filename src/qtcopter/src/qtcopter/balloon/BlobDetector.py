import cv2
import numpy as np


class BlobDetector:
    def __init__(self):
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        #params.minThreshold = 10
        #params.maxThreshold = 200

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.5

        params.filterByColor = False
        params.blobColor = 150

        self._detector = cv2.SimpleBlobDetector(params)

    def find_balloon(self, image):
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        #keypoints = self._detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        #im_with_keypoints = cv2.drawKeypoints(image, keypoints, (0,0,255))
        im_with_keypoints = opening

        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(100)
        return None
