from collections import defaultdict
from queue import PriorityQueue
from pRobustCbss.NodeAndConstClass import negConst
from pRobustCbss.StateForLowLevel import State


class LowLevelPlan:
    def __init__(self, Allocations, Constraint, num_of_cols, num_of_rows, Positions):
        self.Allocations = Allocations                  # Goal allocations for agents
        self.Constraint = Constraint                    # Constraints for agent movements
        self.num_of_cols = num_of_cols                  # Grid columns
        self.num_of_rows = num_of_rows                  # Grid rows
        self.Positions = Positions                      # Initial agent locations

        self.solution = self.run()

    def run(self):
        # Store paths for all agents
        paths = defaultdict(list)
        # Total cost of all paths
        totalCost = 0

        # Process each agent's Goal sequence
        for agent, sequence in self.Allocations.items():
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
            paths[agent] = path
            # Update the total cost
            totalCost += (len(path) - 1)

        return paths, totalCost

    def calc_heuristic_value(self, cur_location, goal):
        # loc1 divided by num_of_cols gives row1, remainder gives col1
        CurRow, CurCol = divmod(cur_location[0], self.num_of_cols)
        # loc2 divided by num_of_cols gives row2, remainder gives col2
        GoalRow, GoalCol = divmod(goal, self.num_of_cols)
        # Compute Manhattan distance as the sum of absolute differences of rows and columns
        return abs(CurRow - GoalRow) + abs(CurCol - GoalCol)

    def GetNeighbors(self, state, agent):
        neighbors = set()
        loc, direct = state.CurPosition

        # Define movement candidates for the agent
        candidates = [loc + 1, loc + self.num_of_cols, loc - 1, loc - self.num_of_cols]

        # Try moving in the current direction
        loc_after_move = candidates[direct]
        if 0 <= loc_after_move < self.num_of_cols * self.num_of_rows and self.validateMove(loc_after_move, agent, state):
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

    def validateMove(self, loc_after_move, agent, state):
        # Check if the move stays within grid boundaries
        if loc_after_move // self.num_of_cols >= self.num_of_rows or loc_after_move % self.num_of_cols >= self.num_of_cols:
            return False

        # Check if the move violates any negative constraints
        for const in self.Constraint:
            if isinstance(const, negConst):
                if const.agent == agent and const.t == state.g + 1:
                    return False

        return True







