from collections import defaultdict
from queue import PriorityQueue
from pRobustCbss.NodeAndConstClass import negConst, posConst
from pRobustCbss.StateForLowLevel import State


class LowLevelPlan:
    def __init__(self, Node, num_of_cols, num_of_rows, Positions, agent_that_need_update_path):
        self.Node = Node                                # Goal allocations for agents
        self.num_of_cols = num_of_cols                  # Grid columns
        self.num_of_rows = num_of_rows                  # Grid rows
        self.Positions = Positions                      # Initial agent locations
        self.agent_that_need_update_path = agent_that_need_update_path

        self.run()

    def run(self):

        # Process each agent's Goal sequence
        for agent in self.agent_that_need_update_path:
            self.Node.g -= (max(1, len(self.Node.paths[agent])) - 1)
            sequence = self.Node.sequence["Allocations"][agent]

            # Priority queue for A* search
            OpenList = PriorityQueue()
            # Initial state for the agent
            S = State(self.Positions[agent])

            # Process each goal in the sequence
            for goal in sequence:
                # Calculate f-value and add to open list
                f_val = self.calc_heuristic_value(S.CurPosition, goal) + S.g
                OpenList.put((f_val, S))

                while not OpenList.empty():
                    # Get the state with the lowest f-value
                    f, S = OpenList.get()

                    # Check if the goal is reached
                    if S.CurPosition[0] == goal:
                        # Reset the open list for the next goal
                        OpenList = PriorityQueue()
                        break

                    # Get neighbors and add them to the open list
                    neighbors = self.GetNeighbors(S, agent)
                    for Sl in neighbors:
                        f_val = self.calc_heuristic_value(Sl.CurPosition, goal) + Sl.g
                        OpenList.put((f_val, Sl))

            # Extract the path from the final goal back to the start
            path = []
            while S is not None:
                path.append(S.CurPosition)
                S = S.parent
            # Reverse the path to start from the agent's initial location
            path.reverse()

            # Append the path for the current goal to the agent's path
            self.Node.paths[agent] = path
            # Update the total cost
            self.Node.g += (len(path) - 1)

    def calc_heuristic_value(self, CurPosition, goal):
        # cur_location divided by num_of_cols gives CurRow, remainder gives CurCol
        CurRow, CurCol = divmod(CurPosition[0], self.num_of_cols)
        # goal divided by num_of_cols gives GoalRow, remainder gives GoalCol
        GoalRow, GoalCol = divmod(goal, self.num_of_cols)
        # Compute Manhattan distance
        time = abs(CurRow - GoalRow) + abs(CurCol - GoalCol)

        if time == 0:
            return 0

        up_or_down = 3 if CurRow > GoalRow else (1 if CurRow < GoalRow else None)
        left_or_right = 2 if CurCol > GoalCol else (0 if CurCol < GoalCol else None)

        if CurPosition[1] == up_or_down or CurPosition[1] == left_or_right:
            return time + 1

        return time + 2

    def GetNeighbors(self, state, agent):
        neighbors = set()
        loc, direct = state.CurPosition

        # Define movement candidates for the agent
        candidates = [loc + 1, loc + self.num_of_cols, loc - 1, loc - self.num_of_cols]

        # Try moving in the current direction
        loc_after_move = candidates[direct]
        if 0 <= loc_after_move < self.num_of_cols * self.num_of_rows and self.validateMove(loc, loc_after_move, agent, state):
            neighbors.add(State((loc_after_move, direct), state.g + 1, state))

        # Try turning left
        new_direction = (direct - 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1, state))

        # Try turning right
        new_direction = (direct + 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1, state))

        # Stay in the same place but increment g (cost)
        neighbors.add(State((loc, direct), state.g + 1, state))

        return neighbors

    def validateMove(self, loc, loc_after_move, agent, state):
        # Check if the move stays within grid boundaries
        if loc_after_move // self.num_of_cols >= self.num_of_rows or loc_after_move % self.num_of_cols >= self.num_of_cols:
            return False

        # Check if the move violates any negative constraints
        for const in self.Node.constraint[agent]:
            if isinstance(const, negConst):
                if const.agent == agent and const.t == state.g + 1 and (const.x == loc_after_move or const.x == (loc, loc_after_move) or const.x == (loc_after_move, loc)):
                    return False

            if isinstance(const, posConst):
                if (const.agent1 == agent or const.agent2 == agent) and const.t == state.g + 1 and (const.x != loc_after_move and const.x != (loc, loc_after_move) and const.x != (loc_after_move, loc)):
                    return False

        return True







