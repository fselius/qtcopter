class SpiralSearch:
    """
    Fly spirals until we found the exact place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['continue spiraling', 'found target']

    def __init__(self, find_func):
        self.__find_func = find_func

    def __call__(self, userdata, output):
        #rois = self.__find_func(userdata.image)
        #if rois is None:
        #    # No ROIs found, continue spiraling
        #    # TODO: publish spiraling delta
        #    return 'continue spiraling'

        #output.rois = rois
        return 'found target'
