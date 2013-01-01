import time
import tf
import rospy

class FlyToMissionFlow:
    """
    Fly to mission site according to given, pre-configured coordinates.
    """

    outcomes = ['on way to mission site', 'mission site reached']

    def __init__(self, offset):
        self.__offset = offset
        self.__sent_delta = False
        self.__listener = tf.TransformListener()
        self.__flow_frame = 'downward_cam_optical_frame'

        # we consider ourselves arrived when we overshoot the target, and when
        # our distance is lower than 1 meter.
        self.__reached_xy_positive = [False, False]
        self.__reached_xy_negative = [False, False]
        
        self.__reached_threshold = 1 # meters when we consider we are close
 
    def __call__(self, userdata, output):
        # TODO: publish deltas, decrease offset and return
        if self.__offset is None:
            return 'mission site reached'
        # 'on way to mission site'
        if not self.__sent_delta:
            x, y, z = self.__offset
            theta = 0

            rospy.loginfo('FlyToMissionFlow sending delta: %r' % ((x, y, z, theta),))
            #userdata.publish_delta(x, y, z, theta)
            userdata.publish_delta__keep_height(x=x,
                                                y=y,
                                                theta=0,
                                                target_height=userdata.max_height)
            self.__sent_delta = True
            time.sleep(0.1)

        try:
            ((dx, dy, dz), drot) = self.__listener.lookupTransform(self.__flow_frame, 'waypoint', rospy.Time(0))
        except:
            rospy.logwarn('Could not lookupTransform in FlyToMissionFlow')
            time.sleep(0.1)
            return 'on way to mission site'
        
        self.__reached_xy_positive[0] |= (dx>0)
        self.__reached_xy_positive[1] |= (dy>0)
        self.__reached_xy_negative[0] |= (dx<0)
        self.__reached_xy_negative[1] |= (dy<0)

        far = not (self.__reached_xy_positive[0]&\
                   self.__reached_xy_positive[1]&\
                   self.__reached_xy_negative[0]&\
                   self.__reached_xy_negative[1])

        far |= not ((dx**2+dy**2)**0.5 < self.__reached_threshold)
        # wait until we reach mission
        if far:
            time.sleep(0.1)
            return 'on way to mission site'
        else:
            return 'mission site reached'

