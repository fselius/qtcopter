class DescendToTarget:
    """
    Fly exactly above the place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['target lost', 'centered above target',
                'drop height reached']

    def __init__(self, find_func):
        self.__find_func = find_func

    def __call__(self, userdata, output):
        rois = self.__find_func(userdata.image)
        if rois is None:
            # Target lost, go back to spiraling.
            return 'target lost'

        if userdata.height.range <= 1.5:
            return 'drop height reached'

        # Fly down and keep above center of target.
        output.rois = rois
        return 'centered above target'
