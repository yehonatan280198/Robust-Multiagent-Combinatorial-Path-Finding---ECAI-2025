import random


class Agent:
    next_id = 1  # Class variable to keep track of the next agent_id

    def __init__(self, delay_by_manufacturer, location, failure_probability):
        self.agent_id = Agent.next_id  # Assign the next available ID
        Agent.next_id += 1  # Increment the next available ID
        self.delay_by_manufacturer = [delay_by_manufacturer, failure_probability]
        self.delay_by_observations = [delay_by_manufacturer, 0]
        self.location = location
        self.planned_path = []
        self.lastMoveTIme = 0

    def get_id(self):
        return self.agent_id

    def set_location(self, location):
        self.location = location

    def get_location(self):
        return self.location

    def set_planned_path(self, path):
        self.planned_path = path

    def add_to_planned_path(self, path):
        self.planned_path += path

    def get_planned_path(self):
        return self.planned_path

    def move(self, units_of_time):
        self.lastMoveTIme = units_of_time

        self.delay_by_observations[1] += 1
        self.delay_by_observations[0] = units_of_time / self.delay_by_observations[1]

        if random.random() < self.delay_by_manufacturer[1]:
            self.delay_by_manufacturer[0] += 1

        next_step = self.planned_path.pop(0)
        self.location = next_step
        return next_step

    def get_nextMove(self):
        return self.planned_path[0]

    def get_delay_by_manufacturer(self):
        return self.delay_by_manufacturer[0]

    def get_delay_by_observations(self):
        return self.delay_by_observations[0]

    def checkIfCanMoveByDelay(self, units_of_time):
        return units_of_time - self.lastMoveTIme >= self.get_delay_by_manufacturer()

    def __str__(self):
        return f"agent: {self.get_id()}"
