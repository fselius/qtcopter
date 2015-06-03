class CenterAbove:
    """
    Fly exactly above the place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['target lost', 'centered above target']

    def __init__(self, find_func):
        self.__find_func = find_func

    def __call__(self, userdata, output):
        rois = self.__find_func(userdata.image)
        if rois is None:
            # No ROIs found, continue spiraling
            # TODO: publish centering delta
            return 'target lost'

        output.rois = rois
        return 'centered above target'
