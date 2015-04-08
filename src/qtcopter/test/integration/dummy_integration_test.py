#!/usr/bin/env python2

"""
Dummy integration test.
"""

PKG = 'qtcopter'
NAME = 'dummy_test'

import rospy
import rostest
import sys
import unittest

import qtcopter

class DummyTest(unittest.TestCase):
    def dummy_test(self):
        print('Dummy integration test!')
        self.AssertEquals(2, 1+1, '1+1 == 2')


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, DummyTest, sys.argv)
