class Label:
    def __init__(self, locations, task_status_vector, cost_vector):
        self.v = tuple(locations)
        self.a = tuple(task_status_vector)
        self.g = tuple(cost_vector)

        self.parent = None
        self.Ic = set()
        self.back_set = set()

        self.f_temp = 0
        self.f_max = 0

        self.joint_sequence = {}
        self.heuristic_vector = {}
        self.joint_policy = {}

