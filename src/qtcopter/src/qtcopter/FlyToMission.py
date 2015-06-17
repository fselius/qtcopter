class FlyToMission:
    """
    Fly to mission site according to given, pre-configured coordinates.
    """

    outcomes = ['on way to mission site', 'mission site reached']

    def __init__(self, offset):
        self.__offset = offset

    def __call__(self, userdata, output):
        # TODO: publish deltas, decrease offset and return
        # 'on way to mission site'
        # userdata.publish_delta(x, y, z, theta)
        return 'mission site reached'
