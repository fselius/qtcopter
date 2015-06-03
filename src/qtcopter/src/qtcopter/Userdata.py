class Userdata:
    """
    Userdata for the state machine states.
    Will raise an exception when trying to access a non-existing
    value or when trying to overwrite an already stored value.
    """

    def __init__(self, other=None):
        if other is not None:
            self.__dict__ = other.__dict__

    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise RuntimeError("Can't overwrite {0}.".format(name))
        self.__dict__[name] = value
