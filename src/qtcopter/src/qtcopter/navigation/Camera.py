#!/usr/bin/env python

'''
This module provides camera functionality:
- Getting camera stats
- Pixel/distance conversion
'''
import rosparam
import rospy
import camera_info_manager
import numpy as np

class Camera(object):
    ''' A class representing a camera. This can be either the current camera,
        or a camera which we took pictures with earlier.
        Currently, this is simply dummy info. Doesn't do actual camera settings change '''
    def __init__(self, camera_info_path=None, camera_info_url=None, cname=''):
        if camera_info_path is not None:
            camera_info_url = 'file://' + camera_info_path
        if camera_info_url is None:
            camera_info_url = rospy.get_param("/camera/camera_nodelet/camera_info_url")
        self._camera_info_url = camera_info_url

        if camera_info_url.upper().startswith('FILE://'):
            self.info = camera_info_manager.loadCalibrationFile(camera_info_url[7:], cname)
        else:
            raise NotImplementedError("only urls of file:// type are supported for now")
            # if we want another url type, rip from here:
            # /opt/ros/indigo/lib/python2.7/dist-packages/camera_info_manager/camera_info_manager.py:
            # CameraInfoManager._loadCalibration
        # make sure it was successful
        if self.info.K[0] == 0.0:
            raise RuntimeError("Could not load camera info. Bad url? (url=%s)" % (self._camera_info_url,))

        self.K = np.array(self.info.K).reshape(3,3)
        self.K_inv = np.linalg.inv(self.K)
    def __repr__(self):
        return '<Camera yaml=%r>' % (self._camera_info_path,)
    def get_ground_offset(self, camera_offset, distance, camera_corner_offset=True):
        ''' get_ground_offset - Get ground offset in meters from camera offset
                                in pixels (pixels should be out of full size)
                                from center.
                camera_offset - pixel offset from center [pixels
                distance      - distance to ground
                camera_corner_offset - whether camera offset is from the top left corner
        '''
        x, y = camera_offset
        # x, y should be corner offsets
        if not camera_corner_offset:
            x, y = x+self.info.width/2., y+self.info.height/2.

        # http://answers.ros.org/question/119506/what-does-projection-matrix-provided-by-the-calibration-represent/
        (x, y, h) = np.dot(self.K_inv, (x, y, 1))*distance
        return x, y
    def get_camera_offset(self, ground_offset, distance, camera_corner_offset=True):
        ''' get_camera_offset - Get camera offset in pixels from ground offset
                                in meters from center.
                ground_offset - ground offset from center [meters]
                distance      - distance to ground [meters]
                camera_corner_offset - return camera offset from top left corner
        '''
        x, y = ground_offset
        x, y, _ = np.dot(self.K, (x, y, distance))
        if not camera_corner_offset:
            x, y = x-self.info.width/2., y-self.info.height/2.
        return x, y

# create instance of default, live camera
default_camera = Camera()

