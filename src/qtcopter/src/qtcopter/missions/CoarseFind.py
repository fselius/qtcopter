from . import MissionState
import rospy
import cv2
from qtcopter.navigation.Camera import default_camera
from .balldrop.utils import center_of_mass
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

class CoarseFind(MissionState):
    def __init__(self, debug_pub, find_roi_func, camera=default_camera):
        MissionState.__init__(self,
                              debug_pub,
                              outcomes=['succeeded',
                                        'aborted'],
                              output_keys=['rois'])
        if find_roi_func is None:
            rospy.logwarn('CoarseFind: Using dummy ROI finder function.')
            self._find_roi = lambda image, **kw: [rect_contour((0, 0, image.shape[1], image.shape[0]))]
        else:
            self._find_roi = find_roi_func
        self._camera = camera
    def publish_offset(self, height, rois):
        # for balldrop, move towards target
        # otherwise, just continue with detailed find
        if not rospy.get_param('camera/pointing_downwards') or height<2:
            return 'succeeded'

        # choose first ROI (should be sorted)
        roi = rois[0]
        image_width = rospy.get_param('camera/image_width')
        image_height = rospy.get_param('camera/image_height')

        # TODO: find ROI center. Currently simply center of bounding rectangle,
        # but we can use center of mass
        rect = cv2.boundingRect(roi)
        center = (rect[0]+rect[2]/2.0, rect[1]+rect[3]/2.0)

        # Get offset in pixels
        offset = (center[0]-image_width/2.0, center[1]-image_height/2.0)
        offset_px = offset[:]

        # Get distance in millimetres
        offset[0] = self._camera.get_ground_offset(offset[0], distance)
        offset[1] = self._camera.get_ground_offset(offset[1], distance)

        # Send a default angle
        angle = tf.transformations.quaternion_from_euler(0, 0, 0)

        if rospy.get_param('camera/pointing_downwards'):
            # Camera points to the ground
            # -> distance to object is z coordinate
            x, y = offset
            z = distance
        else:
            # Camera is facing forward
            # -> distance to object is y coordinate
            x, z = offset
            y = distance

       
        # if target near edge, move to edge
        is_edge_close = lambda value, size: 1.0*value/size<0.1 or 1.0*(size-value)/size<0.1
        close_left  = is_edge_close(rect[0][0], image_width)
        close_top   = is_edge_close(rect[0][1], image_height)
        close_right = is_edge_close(rect[0][0]+rect[1][0], image_width)
        close_bot   = is_edge_close(rect[0][1]+rect[1][1], image_height)
    
        # if ROI is not close to edge..
        if not (close_left or close_top or close_right or close_bot):
            return 'succeeded'
        
        # if center of mass is nearby, don't move
        if abs(offset_px[0]/image_width) < 0.2 and abs(offset_px[1]/image_height) < 0.2:
            return 'succeeded'

        # finally, fly to center of mass
        self._pub.sendTransform((x, y, z),
                        angle,
                        rospy.Time.now(),
                        'camera',
                        'target')
        # continue with coarse find until we are above ROI
        return None
       
    def on_execute(self, userdata, image, height):
        '''
        Runs the "coarse find" algorithm and returns a region of interest.
        '''
        rospy.loginfo('Trying to find coarse ROI in image, height = ' + str(height))
        rois = self._find_roi(image, height=height, camera=self._camera)

        def draw_roi():
            rospy.logdebug('Publishing coarse ROI.')
            for i, cont in enumerate(rois):
                rect = cv2.boundingRect(cont)
                cv2.rectangle(image, rect[0:2], (rect[0]+rect[2], rect[1]+rect[3]), (0, 0, 255))
            return image

        if rois is None or len(rois)==0:
            return None

        self.debug_publish(draw_roi)

        userdata.rois = rois
        return self.publish_offset(height, rois)

