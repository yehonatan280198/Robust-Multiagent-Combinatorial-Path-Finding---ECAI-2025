from collections import defaultdict
from functools import total_ordering


class Node:
    def __init__(self):
        self.paths = defaultdict(lambda: {"path": [], "cost": 0})
        self.negConstraints = defaultdict(set)
        self.posConstraints = defaultdict(set)
        self.g = 0
        self.sequence = {}
        self.isPositiveNode = False

    def __lt__(self, other):
        return self.g < other.g

    def __eq__(self, other):
        return self.g == other.g


@total_ordering
class State:
    def __init__(self, CurLocation, g=0, parent=None, sequence=[]):
        # The current location (position and direction) of the agent
        self.CurLocation = CurLocation
        # Cost to reach this state from the initial state
        self.g = g
        # Parent state for path reconstruction
        self.parent = parent
        self.sequence = sequence

    # Define equality based on current location and cost
    def __eq__(self, other):
        return isinstance(other, State) and self.CurLocation == other.CurLocation and self.g == other.g

    # Define a hash function based on current location and cost
    def __hash__(self):
        return hash((self.CurLocation, self.g))

    # Define less-than for ordering, based on cost g
    def __lt__(self, other):
        return self.g < other.g
