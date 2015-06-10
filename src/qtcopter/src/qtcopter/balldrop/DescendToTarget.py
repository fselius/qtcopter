from ..pub_helpers import draw_roi


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
        #TODO: use userdata.rois of coarse find
        center, size = self.__find_func(userdata.image)
        if center is None:
            # Target lost, go back to spiraling.
            return 'target lost'

        if userdata.height_msg.range <= 1.5:
            return 'drop height reached'

        userdata.publish_debug_image(draw_roi, userdata.rois)

        # Fly down and keep above center of target.
        output.rois = userdata.rois
        return 'centered above target'
