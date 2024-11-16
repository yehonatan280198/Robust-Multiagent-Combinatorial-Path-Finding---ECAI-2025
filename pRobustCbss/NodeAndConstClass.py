
class Node:
    def __init__(self):
        self.paths = []
        self.constraint = set()
        self.g = 0
        self.sequence = {}


class negConst:
    def __init__(self, agent, x, t):
        self.agent = agent
        self.x = x
        self.t = t


class posConst:
    def __init__(self, agent1, agent2, x, t, SumTimeAndDelta):
        self.agent1 = agent1
        self.agent2 = agent2
        self.x = x
        self.t = t
        self.SumTimeAndDelta= SumTimeAndDelta

