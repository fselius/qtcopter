class FlyToBase:
    """
    Fly to base according to accumulated flow sensor measurements.
    """

    outcomes = ['on way to base', 'base reached']

    def __call__(self, userdata, output):
        # TODO: publish deltas and return
        # 'on way to base'
        # userdata.publish_delta(x, y, z, theta)
        return 'base reached'
