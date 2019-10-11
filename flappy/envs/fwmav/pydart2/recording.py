class Recording(object):
    """docstring for Recording"""
    def __init__(self, world):
        super(Recording, self).__init__()
        self.world = world
        self.clear()

    def clear(self, ):
        self.history = list()
        self.bake()

    def num_frames(self, ):
        return len(self.history)

    def bake(self, ):
        data = (self.world.x, self.world.collision_result.copy())
        self.history.append(data)

    def set_frame(self, frame_index):
        if frame_index < 0 or frame_index >= self.num_frames():
            return False
        x, col = self.history[frame_index]
        self.world.set_states(x)
        self.world.collision_result = col.copy()
        return True
