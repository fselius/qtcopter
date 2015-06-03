from . import Userdata, StateMachine
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range, Image
from qtcopter.msg import controller_msg
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospy
import tf


class RosStateMachine(StateMachine):
    def __init__(self, states, transitions, start, outcomes):
        StateMachine.__init__(self, states, transitions, start, outcomes)
        self.__last_output = Userdata()
        self.__bridge = CvBridge()

        # Publishers
        self.__delta_pub = tf.TransformBroadcaster()
        self.__debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)
        self.__pid_input_pub = rospy.Publisher('/pid_input', controller_msg,
                                               queue_size=1)

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

        rospy.logdebug('Received data from subscriptions.')

        u = Userdata(self.__last_output)
        u.publish_debug = self.publish_debug
        u.publish_delta = self.publish_delta
        u.height_msg = range_msg
        try:
            u.image = self.__bridge.imgmsg_to_cv2(image_msg,
                                                  desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logerror(error)
            return
        self.__last_output = self(u)

    def publish_debug(self, image_callback):
        if self._debug_pub.get_num_connections() <= 0:
            return

        image = image_callback()
        img_msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.__debug_pub.publish(img_msg)

    def publish_delta(self, x, y, z, theta):
        rospy.loginfo('Publish delta: x {0}, y {1}, z {2}, theta {3}'
                      .format(x, y, z, theta))
        rotation = tf.transformations.quaternion_from_euler(0, 0, theta)
        # TODO: camera frame (down/forward)
        camera_frame = 'cam'
        self.__delta_pub.sendTransform((x, y, z),
                                       rotation,
                                       rospy.Time.now(),
                                       'waypoint',
                                       camera_frame)
        # TODO
        msg = controller_msg()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.t = theta
        self.__pid_input_pub.publish(msg)
