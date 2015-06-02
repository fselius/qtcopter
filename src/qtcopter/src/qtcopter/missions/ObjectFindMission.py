from qtcopter.missions import CoarseFind, DetailedFind
from sensor_msgs.msg import Image
from qtcopter.msg import controller_msg
import rospy
import smach
import smach_ros
import tf

"""
ROS node for balldrop mission.
"""


class ObjectFindMission(smach.State):
    def __init__(self, name, coarse_find_func, detailed_find_func):
        rospy.init_node(name)

        self.sm = smach.StateMachine(outcomes=['mission successful',
                                               'mission aborted'])
        self.debug_pub = rospy.Publisher('/debug_image', Image, queue_size=1)
        self.delta_pub = tf.TransformBroadcaster()
        self.pid_input_pub = rospy.Publisher('/pid_input', controller_msg, queue_size=1)
        with self.sm:
            smach.StateMachine.add('CoarseFind',
                                   CoarseFind(self.delta_pub, self.pid_input_pub, self.debug_pub,
                                              coarse_find_func),
                                   transitions={'succeeded': 'DetailedFind',
                                                'aborted': 'mission aborted'})
            smach.StateMachine.add('DetailedFind',
                                   DetailedFind(self.delta_pub, self.pid_input_pub, self.debug_pub,
                                                detailed_find_func),
                                   transitions={'failed': 'CoarseFind',
                                                'succeeded': 'mission successful',
                                                'aborted': 'mission aborted'})
        self.sis = smach_ros.IntrospectionServer(name, self.sm, '/SM_ROOT')
        self.sis.start()
        rospy.loginfo('Initialized {0} node.'.format(name))

    def execute(self):
        rospy.loginfo('Executing mission.')
        self.sm.execute()
        self.sis.stop()
