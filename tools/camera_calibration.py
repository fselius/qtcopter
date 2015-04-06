#!/usr/bin/env python2

from argparse import ArgumentParser
import cv2
import logging
import numpy as np


def open_camera(camera_id):
    logging.info('Opening camera %d.' % camera_id)
    camera = cv2.VideoCapture(camera_id)
    if camera is None or not camera.isOpened():
        raise RuntimeError('Could not open camera %d.' % camera_id)

    return lambda: camera.read()


def open_images(images):
    def get_image():
        if len(images) > 0:
            image = images.pop(0)
            return True, cv2.imread(image)
        return False, None
    return get_image


def calibrate_camera(get_image, pattern_shape, square_size, error_threshold):
    pattern_points = np.zeros((np.prod(pattern_shape), 3), np.float32)
    pattern_points[:, 0:2] = np.indices(pattern_shape).T.reshape(-1, 2)
    pattern_points *= square_size

    world_points = []
    image_points = []
    error = float('NaN')
    camera_matrix = None
    dist_coeffs = None

    cv2.namedWindow('Camera calibration', cv2.WINDOW_NORMAL)
    while True:
        success, image = get_image()
        if not success or image is None:
            logging.info('Could not get an image, exiting.')
            break

        success, corners = cv2.findChessboardCorners(image, pattern_shape, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if success:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cv2.cornerSubPix(gray_image, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1))

            image_points.append(corners.reshape(-1, 2))
            world_points.append(pattern_points)

            error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(world_points, image_points, image.shape[0:2], None, None)
            logging.info('Reprojection error: %f' % error)
            logging.info('Camera matrix:')
            logging.info(camera_matrix)
            logging.info('Distortion coefficients:')
            logging.info(dist_coeffs)

            if error < error_threshold:
                break

        cv2.drawChessboardCorners(image, pattern_shape, corners, success)
        cv2.imshow('Camera calibration', image)
        key = cv2.waitKey(1)
        if key == 113:  # 'q'
            logging.warning('User pressed key, exiting early.')
            return

    cv2.destroyWindow('Camera calibration')
    print('Final error: %f' % error)
    print('Camera matrix:')
    print(camera_matrix)
    print('Distortions:')
    print(dist_coeffs)
    return (camera_matrix, dist_coeffs)


def show_undistorted(get_image, camera_matrix, dist_coeffs):
    cv2.namedWindow('Undistorted camera image', cv2.WINDOW_NORMAL)
    while True:
        success, image = get_image()
        if not success or image is None:
            logging.info('Could not get an image, exiting.')
            break

        image = cv2.undistort(image, camera_matrix, dist_coeffs)

        cv2.imshow('Undistorted camera image', image)
        key = cv2.waitKey(1)
        if key == 113:  # 'q'
            logging.warning('User pressed key, exiting early.')
            return


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('--camera-id',
                        help='ID of the attached camera for OpenCV',
                        type=int,
                        default=0)
    parser.add_argument('--images',
                        nargs='+',
                        help='Paths to images to use as input for calibration')
    parser.add_argument('--pattern-shape',
                        help='Inner corners of the used chessboard pattern',
                        default=(9, 6))
    parser.add_argument('-s', '--show-undistorted',
                        action='store_true')
    parser.add_argument('--square-size',
                        help='Size of a square of the chessboard pattern',
                        default=1.0)
    parser.add_argument('--error-threshold',
                        default=1e-2)
    parser.add_argument('-v', '--verbose',
                        action='count',
                        help='Specify multiple times to increase verbosity.',
                        default=0)
    args = parser.parse_args()

    if args.verbose > 1:
        logging.basicConfig(level=logging.INFO)
    elif args.verbose > 0:
        logging.basicConfig(level=logging.DEBUG)

    if args.images:
        image_callback = open_images(args.images)
    else:
        image_callback = open_camera(args.camera_id)
    camera_matrix, dist_coeffs = calibrate_camera(image_callback, args.pattern_shape,
                                                  args.square_size, args.error_threshold)

    if args.show_undistorted:
        show_undistorted(open_camera(args.camera_id), camera_matrix, dist_coeffs)
