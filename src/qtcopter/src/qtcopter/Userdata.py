class Userdata:
    """
    Userdata for the state machine states.
    Will raise an exception when trying to access a non-existing
    value or when trying to overwrite an already stored value.
    """

    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise RuntimeError("Can't overwrite {0}.".format(name))
        self.__dict__[name] = value

    def reset(self):
        """
        Reset the object to accept new data.
        """
        self.__dict__.clear()
