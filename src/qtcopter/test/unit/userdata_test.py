from nose.tools import *
from qtcopter import Userdata

def test_create():
    userdata = Userdata()

def test_get_set():
    u = Userdata()
    u.foo = 42
    assert_equal(42, u.foo)

def test_overwrite():
    u = Userdata()
    u.foo = 42
    assert_raises(RuntimeError, setattr, u, 'foo', 43)

def test_nonexisting():
    u = Userdata()
    assert_raises(AttributeError, getattr, u, 'foo')

def test_reset():
    u = Userdata()
    u.foo = 42
    assert_equal(42, getattr(u, 'foo'))
    u.reset()
    u.foo = 45
    assert_equal(45, getattr(u, 'foo'))

def test_reset_nonexisting():
    u = Userdata()
    u.foo = 42
    assert_equal(42, getattr(u, 'foo'))
    u.reset()
    assert_raises(AttributeError, getattr, u, 'foo')

def test_reset_multiattr():
    u = Userdata()
    u.foo = 42
    assert_equal(42, getattr(u, 'foo'))
    assert_raises(AttributeError, getattr, u, 'bar')
    u.reset()
    u.bar = 45
    assert_raises(AttributeError, getattr, u, 'foo')
    assert_equal(45, getattr(u, 'bar'))
