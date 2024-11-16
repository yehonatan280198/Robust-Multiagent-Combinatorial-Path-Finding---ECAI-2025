from collections import defaultdict
from queue import PriorityQueue

from pRobustCbss.NodeAndConstClass import negConst
from pRobustCbss.StateForLowLevel import State


class LowLevelPlan:
    def __init__(self, Allocations, Constraint, num_of_cols, num_of_rows, Locations):
        self.Allocations = Allocations
        self.Constraint = Constraint
        self.num_of_cols = num_of_cols
        self.num_of_rows = num_of_rows
        self.Locations = Locations

        self.solution = self.run()

    def run(self):
        paths = defaultdict(list)
        totalCost = 0

        for agent, sequence in self.Allocations.items():
            OpenList = PriorityQueue()
            S = State(self.Locations[agent])

            for goal in sequence:

                f_val = self.calc_heuristic_value(S.cur_location, goal) + S.g
                OpenList.put((f_val, S))

                while not OpenList.empty():
                    f, S = OpenList.get()
                    if S.cur_location[0] == goal:
                        OpenList = PriorityQueue()
                        break

                    neighbors = self.GetNeighbors(S, agent)

                    for Sl in neighbors:
                        Sl.parent = S
                        f_val = self.calc_heuristic_value(Sl.cur_location, goal) + Sl.g
                        OpenList.put((f_val, Sl))

            path = []
            while S is not None:
                path.append(S.cur_location)
                S = S.parent
            path.reverse()

            paths[agent] = paths[agent] + path
            totalCost += (len(path) - 1)

        return paths, totalCost

    def calc_heuristic_value(self, cur_location, goal):
        return abs(cur_location[0] // self.num_of_cols - goal // self.num_of_cols) + abs(
            cur_location[0] % self.num_of_cols - goal % self.num_of_cols)

    def GetNeighbors(self, state, agent):
        neighbors = set()
        loc, direct = state.cur_location

        candidates = [loc + 1, loc + self.num_of_cols, loc - 1, loc - self.num_of_cols]
        loc_after_move = candidates[direct]
        if 0 <= loc_after_move < self.num_of_cols * self.num_of_rows and self.validateMove(loc_after_move, agent, state):
            neighbors.add(State((loc_after_move, direct), state.g + 1))

        new_direction = (direct - 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1))

        new_direction = (direct + 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1))

        neighbors.add(State((loc, direct), state.g + 1))

        return neighbors


    def validateMove(self, loc_after_move, agent, state):
        if loc_after_move // self.num_of_cols >= self.num_of_rows or loc_after_move % self.num_of_cols >= self.num_of_cols:
            return False

        for const in self.Constraint:
            if isinstance(const, negConst):
                if const.agent == agent and const.t == state.g + 1:
                    return False

        return True







