from nose.tools import *
from qtcopter import StateMachine


def test_create():
    StateMachine(states=['A'], transitions=[], start='A', outcomes=['finished'])


def test_create_invalid_start():
    a = lambda x, y: a.outcomes[0]
    a.__dict__ = {
        'outcomes': ['finished']
    }
    assert_raises(RuntimeError, StateMachine, states={'A': a},
                  start='B',
                  transitions={},
                  outcomes=['finished'])


def test_state_transfer():
    a = lambda x, y: a.outcomes[0]
    a.__dict__ = {
        'outcomes': ['go to B'],
    }
    b = lambda x, y: a.outcomes[0]
    b.__dict__ = {
        'outcomes': ['finished'],
    }
    sm = StateMachine(states={'A': a, 'B': b},
                      transitions={'go to B': 'B'},
                      start='A',
                      outcomes=['finished'])

    assert_equal('A', sm.current_state)
    sm(None)
    assert_equal('B', sm.current_state)


def test_state_parameter():
    def a(x, y):
        assert_equal(42, x)
        return 'finished'

    a.__dict__ = {
        'outcomes': ['finished']
    }

    sm = StateMachine(states={'A': a},
                      start='A',
                      transitions={},
                      outcomes=['finished'])
    sm(42)


def test_finished():
    a = lambda x, y: a.outcomes[0]
    a.__dict__ = {
        'outcomes': ['finished']
    }
    sm = StateMachine(states={'A': a},
                      start='A',
                      transitions={},
                      outcomes=['finished'])
    sm(None)
    assert_raises(RuntimeError, sm, None)
