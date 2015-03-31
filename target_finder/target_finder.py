#!/usr/bin/env python2

from argparse import ArgumentParser
import cv2
import logging
import numpy as np


def visualize(image, description):
    cv2.imshow(description, image)
    print('Press a key to continue.')
    cv2.waitKey(0)
    cv2.destroyWindow(description)


def find_target(image_path):
    image = cv2.imread(image_path, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    if image is None:
        raise RuntimeError("Could not load image from '%s'." % image_path)
    logging.info("Loaded image from file '%s'." % image_path)
    visualize(image, 'Original image')

    for exp in np.arange(3, -1, -1):
        sigma = 2**exp
        logging.info('Applying Gaussian blur with sigma=%f.' % sigma)
        blur_image = cv2.GaussianBlur(image, (0, 0), sigma)

        t1 = 10
        t2 = 3*t1
        logging.info('Detecting edges with t1=%f, t2=%f.' % (t1, t2))
        edges = cv2.Canny(blur_image, threshold1=t1, threshold2=t2)

        logging.info('Detecting contours.')
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_CCOMP,
                                               cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None:
            logging.info('Found %i contours.' % len(contours))
            vis_image = cv2.cvtColor(blur_image, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(vis_image, contours, -1, (0, 255, 0))
            visualize(vis_image, 'contours')
        else:
            logging.error("No contours found.")

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('image',
                        help='Path to an image of a target.')
    parser.add_argument('-v', '--verbose',
                        action='count',
                        help='Specify multiple times to increase verbosity.',
                        default=0)
    args = parser.parse_args()

    if args.verbose > 1:
        logging.basicConfig(level=logging.INFO)
    elif args.verbose > 0:
        logging.basicConfig(level=logging.DEBUG)

    find_target(args.image)
