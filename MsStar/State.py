class State:
    def __init__(self, cur_locations, cur_task_status):
        self.Vk = tuple(cur_locations)
        self.Ak = tuple(cur_task_status)
        self.parent = None
        self.Ic = set()
        self.back_set = set()
        self.g = 0

    def __eq__(self, other):
        if isinstance(other, State):
            return self.Vk == other.Vk and self.Ak == other.Ak
        return False

    def __hash__(self):
        return hash((self.Vk, self.Ak))

    def __lt__(self, other):
        return self.g < other.g

    def more_dominant_than(self, other):
        if self.Ak == other.Ak and self.g < other.g:
            return True

        if (all(self.Ak[m] >= other.Ak[m] for m in range(len(self.Ak))) and
                any(self.Ak[m] > other.Ak[m] for m in range(len(self.Ak))) and self.g <= other.g):
            return True

        return False
