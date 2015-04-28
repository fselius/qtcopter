import rospy
import smach
import smach_ros
from qtcopter.missions import CoarseFind, DetailedFind

"""
ROS node for balldrop mission.
"""


class ObjectFindMission(smach.State):
    def __init__(self, name, coarse_find_func, detailed_find_func):
        rospy.init_node(name)

        self.sm = smach.StateMachine(outcomes=['mission successful',
                                               'mission aborted'])
        with self.sm:
            smach.StateMachine.add('CoarseFind', CoarseFind(coarse_find_func),
                                   transitions={'succeeded': 'DetailedFind',
                                                'aborted': 'mission aborted'})
            smach.StateMachine.add('DetailedFind', DetailedFind(detailed_find_func),
                                   transitions={'failed': 'CoarseFind',
                                                'succeeded': 'mission successful',
                                                'aborted': 'mission aborted'})
        self.sis = smach_ros.IntrospectionServer(name, self.sm, '/SM_ROOT')
        self.sis.start()
        rospy.loginfo('Initialized mission node.')

    def execute(self):
        rospy.loginfo('Executing mission.')
        self.sm.execute()
        self.sis.stop()
