from collections import deque
from queue import PriorityQueue

from pRobustCbss.NodeStateConstClasses import State


########################################################## Extract path #####################################################3
def extractPath(state):
    path = []
    while state is not None:
        path.insert(0, state.CurPosition)
        state = state.parent

    return path


class LowLevelPlan:
    def __init__(self, dict_of_map_and_dim, Positions, GoalLocations):
        self.MapAndDims = dict_of_map_and_dim
        self.Positions = Positions
        self.GoalLocations = set(GoalLocations)
        self.goal_heuristics = {}

    ########################################################## Low level plan (A*) #####################################################3

    # def run(self, Node, agent_that_need_update_path):
    #     for agent in agent_that_need_update_path:
    #         sequence = Node.sequence["Allocations"][agent]
    #
    #         # If no allocations are present
    #         if len(sequence) == 1:
    #             Node.paths[agent] = [self.Positions[agent]]
    #             continue
    #
    #         Node.g -= (max(1, len(Node.paths[agent])) - 1)
    #
    #         OpenList = PriorityQueue()
    #         S = State(self.Positions[agent])
    #
    #         for goal, direct in sequence[1:]:
    #             visited = set()
    #             OpenList.put((self.h_val(S.CurPosition[0], goal) + S.g, S))
    #
    #             while not OpenList.empty():
    #                 _, S = OpenList.get()
    #                 if S.CurPosition in visited:
    #                     continue
    #
    #                 visited.add(S.CurPosition)
    #
    #                 if S.CurPosition == (goal, direct):
    #                     OpenList = PriorityQueue()
    #                     break
    #
    #                 for Sl in self.GetNeighbors(S, agent, visited, Node):
    #                     OpenList.put((self.h_val(Sl.CurPosition[0], goal) + Sl.g, Sl))
    #
    #         # Extract the path from the final goal back to the start
    #         Node.paths[agent] = extractPath(S)
    #         Node.g += (len(Node.paths[agent]) - 1)

    ########################################################## Calculate heuristic value (BFS) #####################################################3
    def h_val(self, currLoc, goalLoc):
        if (currLoc, goalLoc) in self.goal_heuristics:
            return self.goal_heuristics[(currLoc, goalLoc)]

        counter_of_reach_goals = 0
        visited = set()
        S = State_For_h_val(currLoc)
        queue = deque([S])

        while queue:
            current_state = queue.popleft()
            if current_state.CurLoc in visited:
                continue

            visited.add(current_state.CurLoc)

            if current_state.CurLoc == goalLoc:
                return current_state.g
            # if current_state.CurLoc in self.GoalLocations:
            #     self.goal_heuristics[(currLoc, current_state.CurLoc)] = current_state.g
            #     counter_of_reach_goals += 1
            #     if counter_of_reach_goals == len(self.GoalLocations):
            #         return self.goal_heuristics[(currLoc, goalLoc)]

            for Sl in self.GetNeighborsBFS(current_state):
                if Sl.CurLoc not in visited:
                    queue.append(Sl)

    ########################################################## Get neighbors (BFS) #####################################################3
    def GetNeighborsBFS(self, state):
        neighbors = set()
        loc = state.CurLoc

        for neighborLoc in [loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"]]:
            if self.checkIfNextMoveIsValid(loc, neighborLoc):
                neighbors.add(State_For_h_val(neighborLoc, state.g + 1, state))

        return neighbors

    ########################################################## Check if next move is valid (BFS) #####################################################3
    def checkIfNextMoveIsValid(self, loc, neighborLoc):
        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= neighborLoc < self.MapAndDims["Cols"] * self.MapAndDims["Rows"]):
            return False

        # If the agent is at the right boundary, it cannot move right
        if loc % self.MapAndDims["Cols"] == self.MapAndDims["Cols"] - 1 and neighborLoc % self.MapAndDims[
            "Cols"] == 0:
            return False

        # If the agent is at the left boundary, it cannot move left
        if loc % self.MapAndDims["Cols"] == 0 and neighborLoc % self.MapAndDims["Cols"] == self.MapAndDims[
            "Cols"] - 1:
            return False

        if self.MapAndDims["Map"][neighborLoc] != 0:
            return False

        return True

    ########################################################## Get neighbors (A*) #####################################################3
    def GetNeighbors(self, state, agent, visited, Node):
        neighbors = set()
        loc, direct = state.CurPosition

        # Define movement candidates for the agent
        candidates = [loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"]]

        # Try moving in the current direction
        loc_after_move = candidates[direct]
        canMove = self.validateMove(loc_after_move, agent, state, Node)
        if canMove == 1:
            neighbors.add(State((loc_after_move, direct), state.g + 1, state))

        # Try turning left
        new_direction = (direct - 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1, state))

        # Try turning right
        new_direction = (direct + 1) % 4
        neighbors.add(State((loc, new_direction), state.g + 1, state))

        # Stay in the same place but increment g (cost)
        if canMove == -1:
            neighbors.add(State((loc, direct), state.g + 1, state))
            visited.remove(state.CurPosition)

        return neighbors

    ########################################################## Check if next move is valid (A*) #####################################################3
    def validateMove(self, loc_after_move, agent, state, Node):
        # Extract the agent's location and direction before taking the next step
        loc, _ = state.CurPosition
        max_cells = self.MapAndDims["Cols"] * self.MapAndDims["Rows"]

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < max_cells):
            return 0

        # If the agent is at the right boundary, it cannot move right
        if loc % self.MapAndDims["Cols"] == self.MapAndDims["Cols"] - 1 and loc_after_move % self.MapAndDims[
            "Cols"] == 0:
            return 0

        # If the agent is at the left boundary, it cannot move left
        if loc % self.MapAndDims["Cols"] == 0 and loc_after_move % self.MapAndDims["Cols"] == self.MapAndDims[
            "Cols"] - 1:
            return 0

        if self.MapAndDims["Map"][loc_after_move] != 0:
            return 0

        # Check if the move violates any negative constraints
        for neg_const in Node.negConstraints[agent]:
            if neg_const.t == state.g + 1 and (
                    neg_const.x == loc_after_move or neg_const.x == frozenset((loc, loc_after_move))):
                return -1

        for pos_const in Node.posConstraints[agent]:
            if pos_const.agent1 == agent and pos_const.agent1_time == state.g + 1 and (
                    pos_const.x != loc_after_move and pos_const.x != frozenset((loc, loc_after_move))):
                return -1

            if pos_const.agent2 == agent and pos_const.agent2_time == state.g + 1 and (
                    pos_const.x != loc_after_move and pos_const.x != frozenset((loc, loc_after_move))):
                return -1

        return 1

    def run(self, Node, agent_that_need_update_path):
        # Process each agent's Goal sequence
        for agent in agent_that_need_update_path:
            sequence = Node.sequence["Allocations"][agent]

            # If no allocations are present
            if len(sequence) == 1:
                Node.paths[agent] = [self.Positions[agent]]
                continue

            # Decrease the previous path cost of the current agent
            Node.g -= (max(1, len(Node.paths[agent])) - 1)

            # Priority queue for A* search
            queue = deque([])
            # Initial state for the agent
            S = State(self.Positions[agent])

            # Process each goal in the sequence
            for goal, direct in sequence:
                visited = set()
                queue.append(S)

                while queue:
                    # Get the state with the lowest f-value
                    S = queue.popleft()
                    if S.CurPosition in visited:
                        continue

                    visited.add(S.CurPosition)

                    # Check if the goal is reached
                    if S.CurPosition == (goal, direct):
                        # Reset the open list for the next goal
                        queue = deque([])
                        break

                    # Get neighbors and add them to the open list
                    neighbors = self.GetNeighbors(S, agent, visited, Node)
                    for Sl in neighbors:
                        queue.append(Sl)

            # Extract the path from the final goal back to the start
            Node.paths[agent] = extractPath(S)
            Node.g += (len(Node.paths[agent]) - 1)

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


class State_For_h_val:

    def __init__(self, CurLoc, g=0, parent=None):
        # The current location (position and direction) of the agent
        self.CurLoc = CurLoc
        # Cost to reach this state from the initial state
        self.g = g
        # Parent state for path reconstruction
        self.parent = parent

    # Define equality based on current location and cost
    def __eq__(self, other):
        return isinstance(other, State_For_h_val) and self.CurLoc == other.CurLoc and self.g == other.g

    # Define a hash function based on current location and cost
    def __hash__(self):
        return hash((self.CurLoc, self.g))

    # Define less-than for ordering, based on cost g
    def __lt__(self, other):
        return self.g < other.g

