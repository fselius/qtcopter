from ..pub_helpers import draw_circle


class DescendToTarget:
    """
    Fly exactly above the place of the mission, as indicated
    by our finder function.
    """

    outcomes = ['target lost', 'centered above target',
                'drop height reached']

    def __init__(self, find_func, drop_height):
        self.__find_func = find_func
        self.__drop_height = drop_height

    def __call__(self, userdata, output):
        center, diameter = self.__find_func(userdata.image)
        if center is None:
            # Target lost, go back to spiraling.
            return 'target lost'

        userdata.publish_debug_image(draw_circle, center, diameter)

        offset = userdata.camera.get_ground_offset(center, userdata.height_msg.range)
        userdata.publish_delta__keep_height(x=offset[0], y=offset[1], theta=0,
                                            target_height=self.__drop_height)

        # If centered and at drop height:
        # TODO: configure offset
        # TODO: check height condition with epsilon
        if abs(offset[0]) < 0.05 and abs(offset[1]) < 0.05 and userdata.height_msg.range <= self.__drop_height:
            return 'drop height reached'

        # Fly down and keep above center of target.
        return 'centered above target'
