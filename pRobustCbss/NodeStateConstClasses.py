from collections import defaultdict
from functools import total_ordering


class Node:
    def __init__(self):
        self.paths = defaultdict(list)
        self.negConstraints = defaultdict(set)
        self.posConstraints = defaultdict(set)
        self.g = 0
        self.sequence = {}

    def __lt__(self, other):
        return self.g < other.g


class negConst:
    def __init__(self, agent, x, t):
        self.agent = agent
        self.x = x
        self.t = t

    def __eq__(self, other):
        if not isinstance(other, negConst):
            return False
        return (self.agent, self.x, self.t) == (other.agent, other.x, other.t)

    def __hash__(self):
        return hash((self.agent, self.x, self.t))


class posConst:
    def __init__(self, agent1, agent2, x, agent1_time, agent2_time):
        self.agent1 = agent1
        self.agent2 = agent2
        self.x = x
        self.agent1_time = agent1_time
        self.agent2_time = agent2_time

    def __eq__(self, other):
        if not isinstance(other, posConst):
            return False
        return (self.agent1, self.agent2, self.x, self.agent1_time, self.agent2_time) == (
            other.agent1, other.agent2, other.x, other.agent1_time, other.agent2_time)

    def __hash__(self):
        return hash((self.agent1, self.agent2, self.x, self.agent1_time, self.agent2_time))


@total_ordering
class State:
    def __init__(self, CurPosition, g=0, parent=None):
        # The current location (position and direction) of the agent
        self.CurPosition = CurPosition
        # Cost to reach this state from the initial state
        self.g = g
        # Parent state for path reconstruction
        self.parent = parent

    # Define equality based on current location and cost
    def __eq__(self, other):
        return isinstance(other, State) and self.CurPosition == other.CurPosition and self.g == other.g

    # Define a hash function based on current location and cost
    def __hash__(self):
        return hash((self.CurPosition, self.g))

    # Define less-than for ordering, based on cost g
    def __lt__(self, other):
        return self.g < other.g
