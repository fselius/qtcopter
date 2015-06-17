from nose.tools import *
from qtcopter import Userdata


def test_create():
    Userdata()


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


def test_create_existing():
    u = Userdata()
    u.foo = 42
    u2 = Userdata(u)
    assert_equal(42, getattr(u2, 'foo'))


def test_create_existing_overwrite():
    u = Userdata()
    u.foo = 42
    u2 = Userdata(u)
    assert_raises(RuntimeError, setattr, u2, 'foo', 43)


def test_contains():
    u = Userdata()
    assert_false('foo' in u)
    u.foo = 42
    assert_true('foo' in u)
