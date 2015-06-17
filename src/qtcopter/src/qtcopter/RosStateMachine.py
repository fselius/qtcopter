from . import Userdata, StateMachine, Camera, OpticalFlow
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range, Image
from qtcopter.msg import controller_msg
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospy
import tf


class RosStateMachine(StateMachine):
    def __init__(self, states, transitions, start, outcomes, camera=None,
                 optical_flow=None):
        StateMachine.__init__(self, states, transitions, start, outcomes)
        self.__last_output = Userdata()
        self.__bridge = CvBridge()

        # Publishers
        self.__delta_pub = tf.TransformBroadcaster()
        self.__debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)
        self.__pid_input_pub = rospy.Publisher('/pid_input', controller_msg,
                                               queue_size=1)

        if camera is None:
            self.__camera = Camera.from_ros()
        else:
            self.__camera = camera

        if optical_flow is None:
            self.__optical_flow = OpticalFlow.from_ros()
        else:
            self.__optical_flow = optical_flow

        self.__config_max_height = rospy.get_param('config/max_height')

    def execute(self):
        self.__height_sub = Subscriber('height', Range)
        self.__image_sub = Subscriber('image', Image)
        self.__sync = ApproximateTimeSynchronizer([self.__height_sub,
                                                   self.__image_sub],
                                                  queue_size=1,
                                                  slop=0.05)
        self.__sync.registerCallback(self.__callback)

        rospy.spin()

    def __callback(self, range_msg, image_msg):
        """
        Receives ROS messages and advances the state machine.
        """
        if self.current_state == self.FINAL_STATE:
            rospy.loginfo('Final state reached.')
            rospy.signal_shutdown('Final state reached.')
            self.__height_sub.unregister()
            self.__image_sub.unregister()
            del self.__sync
            return

        u = self.create_userdata(range_msg=range_msg,
                                 image_msg=image_msg)
        self.__last_output = self(u)

    def publish_debug(self, userdata, image_callback, *args, **kwargs):
        if self.__debug_pub.get_num_connections() <= 0:
            return

        image = image_callback(userdata, *args, **kwargs)
        if image is None:
            return

        img_msg = self.__bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.__debug_pub.publish(img_msg)

    def publish_delta(self, x, y, z, theta):
        rospy.loginfo('Publish delta: x {0}, y {1}, z {2}, theta {3}'
                      .format(x, y, z, theta))
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        # TODO: camera frame (down/forward)
        camera_frame = 'cam'
        self.__delta_pub.sendTransform((x, y, z),
                                       q,
                                       rospy.Time.now(),
                                       'waypoint',
                                       camera_frame)
        # TODO: remove legacy topic publish
        msg = controller_msg()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.t = theta
        self.__pid_input_pub.publish(msg)

    def publish_delta__keep_height(self, userdata, x, y, theta, target_height):
        delta_z = target_height - userdata.height_msg.range
        self.publish_delta(x, y, delta_z, theta)

    def create_userdata(self, **kwargs):
        u = Userdata(self.__last_output)
        u.publish_debug_image = lambda cb, *args, **kwargs: self.publish_debug(u.image, cb, *args, **kwargs)
        u.publish_delta = self.publish_delta
        u.publish_delta__keep_height = lambda x, y, theta, target_height: self.publish_delta__keep_height(u, x, y, theta, target_height)
        u.height_msg = kwargs['range_msg']
        u.max_height = self.__config_max_height
        u.camera = self.__camera
        u.optical_flow = self.__optical_flow
        try:
            u.image = self.__bridge.imgmsg_to_cv2(kwargs['image_msg'],
                                                  desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logerror(error)
            return None

        return u
