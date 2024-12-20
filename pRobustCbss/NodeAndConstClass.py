from collections import defaultdict


class Node:
    def __init__(self):
        self.paths = defaultdict(list)
        self.constraint = defaultdict(set)
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
    def __init__(self, agent1, agent2, x, t, SumTimeAndDelta):
        self.agent1 = agent1
        self.agent2 = agent2
        self.x = x
        self.t = t
        self.SumTimeAndDelta = SumTimeAndDelta

    def __eq__(self, other):
        if not isinstance(other, posConst):
            return False
        return (self.agent1, self.agent2, self.x, self.t, self.SumTimeAndDelta) == (other.agent1, other.agent2, other.x, other.t, other.SumTimeAndDelta)

    def __hash__(self):
        return hash((self.agent1, self.agent2, self.x, self.t, self.SumTimeAndDelta))

