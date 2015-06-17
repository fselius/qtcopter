#!/usr/bin/env python2

from qtcopter import RosStateMachine as StateMachine
from nose.tools import *
import rostest
import unittest
import rospy

PKG = 'qtcopter'
NAME = 'state_machine_transition_test'


class FirstState:
    outcomes = ['go to second state']

    def __call__(self, userdata, output):
        output.test_result = 42
        return 'go to second state'


class SecondState:
    outcomes = ['finished']

    def __call__(self, userdata, output):
        assert_equal(42, userdata.test_result)
        return 'finished'


class TransitionTest(unittest.TestCase):
    def test_transition(self):
        rospy.init_node(NAME)
        sm = StateMachine(states={'A': FirstState(),
                                  'B': SecondState()},
                          transitions={'go to second state': 'B'},
                          start='A',
                          outcomes=['finished'])
        sm.execute()
        assert_true(sm.is_finished())


if __name__ == '__main__':
    rostest.run(PKG, NAME, TransitionTest)
