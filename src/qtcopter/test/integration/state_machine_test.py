#!/usr/bin/env python2

from qtcopter import RosStateMachine as StateMachine
from nose.tools import *
import rostest
import unittest
import rospy

PKG = 'qtcopter'
NAME = 'state_machine_test'


class TestState:
    outcomes = ['finished']

    def __call__(self, userdata, output):
        assert_equal(3.0, userdata.height_msg.range)
        return 'finished'


class StateMachineTest(unittest.TestCase):
    def test_single_state(self):
        rospy.init_node(NAME)
        sm = StateMachine(states={'A': TestState()},
                          transitions={},
                          start='A',
                          outcomes=['finished'])
        sm.execute()
        assert_true(sm.is_finished())


if __name__ == '__main__':
    rostest.run(PKG, NAME, StateMachineTest)
