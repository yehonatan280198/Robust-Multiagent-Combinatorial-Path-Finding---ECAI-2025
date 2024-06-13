class Agent:
    def __init__(self, delay_by_manufacturer, location, color, agent_id):
        self.delay_by_manufacturer = delay_by_manufacturer
        self.agent_id = agent_id
        self.location = location
        self.goals = []
        self.planned_path = []
        self.color = color

    def get_id(self):
        return self.agent_id

    def set_location(self, location):
        self.location = location

    def set_color(self, color):
        self.color = color

    def get_color(self):
        return self.color

    def get_location(self):
        return self.location

    def add_goal(self, goal):
        self.goals.append(goal)

    def get_goals(self):
        return self.goals

    def set_planned_path(self, path):
        self.planned_path = path

    def get_planned_path(self):
        return self.planned_path

    def move(self):
        next_step = self.planned_path.pop(0)
        self.location = next_step
        if next_step in self.goals:
            self.goals.remove(next_step)
            return True, next_step
        return False, next_step

    def get_delay_by_manufacturer(self):
        return self.delay_by_manufacturer
