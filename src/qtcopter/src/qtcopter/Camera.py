#!/usr/bin/env python

'''
This module provides camera functionality:
- Getting camera stats
- Pixel/distance conversion
'''
import numpy as np
from time import sleep

class Camera:
    ''' A class representing a camera. This can be either the current camera,
        or a camera which we took pictures with earlier.
        Currently, this is simply dummy info. Doesn't do actual camera settings change '''

    def __init__(self, camera_name, camera_info):
        self.camera_name = camera_name
        self.info = camera_info
        self.K = np.array(self.info.K).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)

        # TODO: Get actual resolution. It may be different than the default one.
        self.height = self.info.height
        self.width = self.info.width

    def __repr__(self):
        return '<Camera name={0}>'.format(self.camera_name)

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
            x, y = x+self.width/2., y+self.height/2.

        # http://answers.ros.org/question/119506
        (x, y, h) = np.dot(self.K_inv, (x, y, 1))*distance
        return x/h, y/h

    def get_camera_offset(self, ground_offset, distance, camera_corner_offset=True):
        ''' get_camera_offset - Get camera offset in pixels from ground offset
                                in meters from center.
                ground_offset - ground offset from center [meters]
                distance      - distance to ground [meters]
                camera_corner_offset - return camera offset from top left corner
        '''
        x, y = ground_offset
        x, y, h = np.dot(self.K, (x, y, distance))
        x = x/h
        y = y/h
        if not camera_corner_offset:
            x, y = x-self.width/2., y-self.height/2.
        return x, y

    @classmethod
    def from_ros(cls):
        import rospy
        from sensor_msgs.msg import CameraInfo, Image

        camera_name = rospy.get_param('camera/name')
        rospy.loginfo('Initializing camera {0} from ROS.'.format(camera_name))
        while True:
            try:
                camera_info = rospy.wait_for_message('/camera/camera_info', CameraInfo, timeout=0.1)
            except:
                # timeout..
                rospy.loginfo('Initializing.. camera {0} from ROS.'.format(camera_name))
                img = rospy.wait_for_message('/image', Image)
            else:
                break

        # get dummy frame to make camera send camera_info
        rospy.loginfo('Initialized camera {0} from ROS.'.format(camera_name))

        assert camera_name == camera_info.header.frame_id, "Camera name in config '{0}' does not match running camera node '{1}'.".format(camera_name, camera_info.header.frame_id)
        return cls(camera_name, camera_info)

    @classmethod
    def from_file(cls):
        import rospy
        import camera_info_manager as cim

        camera_info_url = rospy.get_param('camera/info_url')
        camera_name = rospy.get_param('camera/name')

        rospy.loginfo('Initializing camera {0} from {1}.'.format(
                      camera_name,
                      camera_info_url))

        if camera_info_url.upper().startswith('FILE://'):
            camera_info = cim.loadCalibrationFile(camera_info_url[7:],
                                                  camera_name)
        else:
            raise NotImplementedError("Only urls of file:// type are supported for now")
            # if we want another url type, rip from here:
            # /opt/ros/indigo/lib/python2.7/dist-packages/camera_info_manager/camera_info_manager.py:
            # CameraInfoManager._loadCalibration

        # make sure it was successful
        if camera_info.K[0] == 0.0:
            raise RuntimeError("Could not load camera info. Bad url? (url={0})".format(camera_info_url))

        return cls(camera_name, camera_info)
