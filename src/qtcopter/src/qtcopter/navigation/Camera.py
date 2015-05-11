#!/usr/bin/env python

'''
This module provides camera functionality:
- Getting camera stats
- Pixel/distance conversion
'''

CAMERAS = {}
CAMERAS['CMLN-13S2C-CS'] = {
    'full_name': 'PTGrey Chameleon CMLN-13S2C-CS',
    'max_resolution': (1296, 964), # (1280, 960) ?
    'max_resolution_fps': 18,
    'resolutions': [(1296, 964), (1280, 960)],
    'width_dest_ratio': 0.81/1280, # in 1280 pixels we saw 81cm from 1m distance
    'height_dest_ratio': 0.81/1280,
}
CAMERAS['iphone 6 plus'] = {
    'full_name': 'iPhone 6 Plus',
    'max_resolution': (3264, 2448), # (1280, 960) ?
    'max_resolution_fps': 60,
    'resolutions': [(3264, 2448), (2448, 2448)],
    'width_dest_ratio': 1.13/3264, # in 3264 pixels we saw 113cm from 1m distance
    'height_dest_ratio': 1.13/3264, # =~ 0.85/2/2448
}
DEFAULT_CAMERA = 'iphone 6 plus'

class Camera(object):
    ''' A class representing a camera. This can be either the current camera,
        or a camera which we took pictures with earlier.
        Currently, this is simply dummy info. Doesn't do actualy camera setting schange '''
    def __init__(self, cam=DEFAULT_CAMERA, dummy=False):
        self.info = CAMERAS[cam]
        self.dummy = dummy
        self.resolution = self.info['max_resolution']
        self.sensor_rect = self.resolution
        #self.size_ratio = (1, 1) # width, height
    def get_resolution(self):
        " Get camera resolution "
        return self.resolution
    def set_resolution(self, resolution, sensor_rect=None):
        ''' Set resolution
        Parameters:
            resolution - resolution
            sensor_rect - what part of the sensor to take. default - all
                          x, y, width, height
        '''
        if not self.dummy:
            raise NotImplementedError()
        self.resolution = resolution
        self.sensor_rect = sensor_rect
        
    def get_full_resolution(self):
        " Get maximum resolution "
        return self.info['max_resolution']
    
    def get_ground_offset(self, camera_offset, distance, camera_corner_offset=True):
        ''' get_ground_offset - Get ground offset in meters from camera offset
                                in pixels (pixels should be out of full size)
                                from center.
                camera_offset - pixel offset from center [pixels
                distance      - distance to ground
                camera_corner_offset - whether camera offset is from the top left corner
        '''
        x, y = camera_offset
        # convert to offset from top left corner
        if not camera_corner_offset:
            x, y = x+self.resolution[0]/2, y+self.resolution[1]/2
        # convert to offset in camera sensor
        # sensor_rect.X + x/res * sensor_rect.width
        x = self.sensor_rect[0] + float(x) / self.resolution[0] * self.sensor_rect[2]
        y = self.sensor_rect[1] + float(y) / self.resolution[1] * self.sensor_rect[3]
        # convert to offset from middle of camera sensor
        x, y = x-self.resolution[0]/2, y-self.resolution[1]/2
        # finally, calculate ratio
        return x*self.info['width_dest_ratio']*distance, y*self.info['height_dest_ratio']*distance
    def get_camera_offset(self, ground_offset, distance, camera_corner_offset=False):
        ''' get_camera offset - Get camera offset in pixels from ground offset
                                in meters from center.
                ground_offset - ground offset from center [meters]
                distance      - distance to ground
                camera_corner_offset - whether camera offset is from the top left corner
        '''
        x, y = ground_offset
        x, y = x/(self.info['width_dest_ratio']*distance), y/(self.info['height_dest_ratio']*distance)
        if camera_corner_offset:
            x, y = x+self.info['max_resolution'][0]/2, y+self.info['max_resolution'][1]/2
        return x, y

camera = Camera()
