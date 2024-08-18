import MAPF

from typing import Dict, List, Tuple, Set
from queue import PriorityQueue
import numpy as np


# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:
    def __init__(self, pyenv=None):
        if pyenv is not None:
            self.env = pyenv.env
        print("pyMAPFPlanner created!  python debug")

    def initialize(self, preprocess_time_limit: int):
        print("planner initialize done... python debug")
        return True

    def plan(self, time_limit):
        return self.sample_priority_planner(time_limit)

    def getManhattanDistance(self, loc1: int, loc2: int):
        return abs(loc1 // self.env.cols - loc2 // self.env.cols) + abs(loc1 % self.env.cols - loc2 % self.env.cols)

    def validateMove(self, loc: int):
        return not (loc // self.env.cols >= self.env.rows or loc % self.env.cols >= self.env.cols or self.env.map[
            loc] == 1)

    def getNeighbors(self, location: int, direction: int):
        neighbors = []

        # forward
        candidates = [location + 1, location + self.env.cols, location - 1, location - self.env.cols]
        if 0 <= candidates[direction] < len(self.env.map) and self.validateMove(candidates[direction]):
            neighbors.append((candidates[direction], direction))

        # turn left
        new_direction = direction - 1
        if new_direction == -1:
            new_direction = 3
        neighbors.append((location, new_direction))

        # turn right
        new_direction = direction + 1
        if new_direction == 4:
            new_direction = 0
        neighbors.append((location, new_direction))

        return neighbors

    def space_time_plan(self, start: int, start_direct: int, goals, reservation: Set[Tuple[int, int, int]]):
        allPath = []
        for end, _ in goals:
            print(start, start_direct, end)
            path = []
            open_list = PriorityQueue()
            all_nodes = {}  # loc+dict, t
            parent = {}
            s = (start, start_direct, 0, self.getManhattanDistance(start, end))
            open_list.put((s[3], id(s), s))
            parent[(start * 4 + start_direct, 0)] = None

            while not open_list.empty():
                _, _, curr = open_list.get()
                curr_location, curr_direction, curr_g, _ = curr

                if (curr_location * 4 + curr_direction, curr_g) in all_nodes:
                    continue

                all_nodes[(curr_location * 4 + curr_direction, curr_g)] = curr
                if curr_location == end:
                    while True:
                        path.append((curr[0], curr[1]))
                        curr = parent[(curr[0] * 4 + curr[1], curr[2])]
                        if curr is None:
                            break

                    path.pop()
                    path.reverse()
                    break

                neighbors = self.getNeighbors(curr_location, curr_direction)

                for neighbor in neighbors:
                    neighbor_location, neighbor_direction = neighbor

                    if (neighbor_location, -1, curr[2] + 1) in reservation:
                        continue

                    if (neighbor_location, curr_location, curr[2] + 1) in reservation:
                        continue

                    neighbor_key = (neighbor_location * 4 +
                                    neighbor_direction, curr[2] + 1)

                    if neighbor_key in all_nodes:
                        old = all_nodes[neighbor_key]
                        if curr_g + 1 < old[2]:
                            old = (old[0], old[1], curr_g + 1, old[3], old[4])
                    else:
                        next_node = (neighbor_location, neighbor_direction, curr_g + 1,
                                     self.getManhattanDistance(neighbor_location, end))

                        open_list.put(
                            (next_node[3] + next_node[2], id(next_node), next_node))

                        parent[(neighbor_location * 4 +
                                neighbor_direction, next_node[2])] = curr

            allPath.append(path)
            start, start_direct = path[-1][0], path[-1][1]

        allPath = [item for sublist in allPath for item in sublist]
        for v in allPath:
            print(f"({v[0]},{v[1]}), ", end="")
        print()
        return allPath

    def sample_priority_planner(self, time_limit: int):

        actions = [MAPF.Action.W] * len(self.env.curr_states)
        reservation = set()  # loc1, loc2, t
        AllPathSize = []

        for i in range(self.env.num_of_agents):
            print("start plan for agent", i)
            path = []

            if not self.env.goal_locations[i]:
                actions[i] = MAPF.Action.NA
                AllPathSize.append(0)
                print("Which does not have any goal left.")
                path.append((self.env.curr_states[i].location, self.env.curr_states[i].orientation))
                reservation.add((self.env.curr_states[i].location, -1, 1))

            else:
                print("with start and goal:")
                path = self.space_time_plan(
                    self.env.curr_states[i].location,
                    self.env.curr_states[i].orientation,
                    # self.env.goal_locations[i][0][0],
                    self.env.goal_locations[i],
                    reservation
                )
                AllPathSize.append(self.env.observationDelay_TotalMoves[i][0] * len(path))

            if path:
                print("current location:", path[0][0], "current direction:", path[0][1])
                if path[0][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif path[0][1] != self.env.curr_states[i].orientation:
                    incr = path[0][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

                last_loc = -1
                t = 1
                for p in path:
                    reservation.add((p[0], -1, t))
                    if last_loc != -1:
                        reservation.add((last_loc, p[0], t))
                    last_loc = p[0]
                    t += 1

        self.env.makeSpanForCurPlan = max(AllPathSize)
        return actions


if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
