from queue import PriorityQueue
from pRobustCbss.NodeStateConstClasses import negConst, posConst
from pRobustCbss.NodeStateConstClasses import State


class LowLevelPlan:
    def __init__(self, Node, dict_of_map_and_dim, Positions, agent_that_need_update_path):
        self.Node = Node                                                                            # Goal allocations for agents
        self.dict_of_map_and_dim = dict_of_map_and_dim
        self.Positions = Positions                                                                  # Initial agent locations
        self.agent_that_need_update_path = agent_that_need_update_path
        self.goal_heuristics = {}

        # self.rotate = rotate

        self.run()

    def run(self):

        # Process each agent's Goal sequence
        for agent in self.agent_that_need_update_path:
            sequence = self.Node.sequence["Allocations"][agent]

            # If no allocations are present
            if len(sequence) == 1:
                self.Node.paths[agent] = [self.Positions[agent]]
                continue

            # Decrease the previous path cost of the current agent
            self.Node.g -= (max(1, len(self.Node.paths[agent])) - 1)

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
                    _, S = OpenList.get()

                    # Check if the goal is reached
                    if S.CurPosition[0] == goal:
                        # Reset the open list for the next goal
                        OpenList = PriorityQueue()
                        break

                    # Get neighbors and add them to the open list
                    neighbors = self.GetNeighbors(S, agent)
                    # else:
                    #     neighbors = self.GetNeighborsOriginal(S, agent)
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

        # Check if the heuristic value for this position and goal has already been calculated
        item = (CurPosition, goal)
        if item in self.goal_heuristics:
            return self.goal_heuristics[item]

        # cur_location divided by num_of_cols gives CurRow, remainder gives CurCol
        CurRow, CurCol = divmod(CurPosition[0], self.dict_of_map_and_dim["Cols"])
        # goal divided by num_of_cols gives GoalRow, remainder gives GoalCol
        GoalRow, GoalCol = divmod(goal, self.dict_of_map_and_dim["Cols"])
        # Compute Manhattan distance
        time = abs(CurRow - GoalRow) + abs(CurCol - GoalCol)

        # If the current position is already at the goal, the heuristic is 0
        if time == 0:
            self.goal_heuristics[item] = 0
            return 0

        # Determine the direction the agent needs to move to reach the goal
        up_or_down = 3 if CurRow > GoalRow else (1 if CurRow < GoalRow else -1)
        left_or_right = 2 if CurCol > GoalCol else (0 if CurCol < GoalCol else -1)

        # If the agent is neither in the same row nor the same column as the goal, and is facing a direction that moves it closer to the goal
        if (CurPosition[1] == up_or_down or CurPosition[1] == left_or_right) and up_or_down != -1 and left_or_right != -1:
            self.goal_heuristics[item] = time + 1
            return time + 1

        # If the agent is in the same row or column as the goal
        elif up_or_down == -1 or left_or_right == -1:
            goalDirect = max(up_or_down, left_or_right)
            countRotate = abs(goalDirect - CurPosition[1]) if abs(goalDirect - CurPosition[1]) != 3 else 1
            self.goal_heuristics[item] = time + countRotate
            return time + countRotate

        # The agent needs to change direction twice to reach the goal, add 2 to the Manhattan distance
        self.goal_heuristics[item] = time + 2
        return time + 2

    def GetNeighbors(self, state, agent):
        neighbors = set()
        loc, direct = state.CurPosition

        # Define movement candidates for the agent
        candidates = [loc + 1, loc + self.dict_of_map_and_dim["Cols"], loc - 1, loc - self.dict_of_map_and_dim["Cols"]]

        # Try moving in the current direction
        loc_after_move = candidates[direct]
        if self.validateMove(loc_after_move, agent, state):
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
        # Extract the agent's location and direction before taking the next step
        loc, _ = state.CurPosition

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < self.dict_of_map_and_dim["Cols"] * self.dict_of_map_and_dim["Rows"]):
            return False

        # If the agent is at the right boundary, it cannot move right
        if loc % self.dict_of_map_and_dim["Cols"] == self.dict_of_map_and_dim["Cols"] - 1 and loc_after_move % self.dict_of_map_and_dim["Cols"] == 0:
            return False

        # If the agent is at the left boundary, it cannot move left
        if loc % self.dict_of_map_and_dim["Cols"] == 0 and loc_after_move % self.dict_of_map_and_dim["Cols"] == self.dict_of_map_and_dim["Cols"] - 1:
            return False

        if self.dict_of_map_and_dim["Map"][loc_after_move] != 0:
            return False

        # Check if the move violates any negative constraints
        for const in self.Node.constraint[agent]:
            if isinstance(const, negConst):
                if const.t == state.g + 1 and (const.x == loc_after_move or const.x == (loc, loc_after_move) or const.x == (loc_after_move, loc)):
                    return False

            elif isinstance(const, posConst):
                if const.agent1 == agent and const.t == state.g + 1 and (const.x != loc_after_move and const.x != (loc, loc_after_move) and const.x != (loc_after_move, loc)):
                    return False

                if const.agent2 == agent and const.SumTimeAndDelta == state.g + 1 and (const.x != loc_after_move and const.x != (loc, loc_after_move) and const.x != (loc_after_move, loc)):
                    return False

        return True

    # def GetNeighborsOriginal(self, state, agent):
    #     neighbors = set()
    #     loc, direct = state.CurPosition
    #
    #     candidates = [loc + 1, loc + self.dict_of_map_and_dim["Cols"], loc - 1, loc - self.dict_of_map_and_dim["Cols"]]
    #
    #     for loc_after_move in candidates:
    #         if self.validateMove(loc_after_move, agent, state):
    #             neighbors.add(State((loc_after_move, direct), state.g + 1, state))
    #
    #     return neighbors











