import math
from collections import defaultdict
import numpy as np
from collections import deque
import pulp


class kBestSequencingByService:

    def __init__(self, AgentLocations, GoalLocations, dict_of_map_and_dim):
        self.num_agents = len(AgentLocations)
        self.num_goals = len(GoalLocations)
        self.total_nodes = self.num_agents + self.num_goals
        self.all_nodes = AgentLocations + GoalLocations
        self.goal_indices = list(range(self.num_agents, self.total_nodes))
        self.MapAndDims = dict_of_map_and_dim

        self.cost_dict = self.precompute_costs(GoalLocations)
        self.cost_matrix = self.build_cost_matrix()

        # Create the MILP model with a minimization objective
        self.model = pulp.LpProblem("MinimizeTotalServiceTime", pulp.LpMinimize)

        # Binary variables x[i,j]: whether there is a path from node i to node j
        self.x = pulp.LpVariable.dicts(
            "x",
            ((i, j) for i in range(self.total_nodes) for j in self.goal_indices if i != j),
            cat="Binary")

        # Integer variables t[j]: service time at goal j
        self.t = pulp.LpVariable.dicts(
            "t",
            (j for j in self.goal_indices),
            lowBound=0,
            cat="Integer")

        # Objective: minimize the total service time across all goals
        self.model += pulp.lpSum([self.t[j] for j in self.goal_indices])

        # Constraints
        for j in self.goal_indices:

            # Constraint 1: each goal must have exactly one incoming edge (visited once)
            self.model += pulp.lpSum([self.x[i, j] for i in range(self.total_nodes) if i != j]) == 1

            # Constraint 2: each goal can lead to at most one other goal
            self.model += pulp.lpSum([self.x[j, k] for k in self.goal_indices if k != j]) <= 1

        # Constraint 3: each agent starts at most one path
        for a in range(self.num_agents):
            self.model += pulp.lpSum([self.x[a, j] for j in self.goal_indices]) <= 1

        # Constraint 4: timing constraints based on transitions
        for i in range(self.total_nodes):
            for j in self.goal_indices:
                if i != j:
                    if i < self.num_agents:
                        # If coming directly from an agent to a goal
                        self.model += self.t[j] >= self.cost_matrix[i][j] - (1 - self.x[i, j]) * 10000000
                    else:
                        # If coming from a previous goal to the current goal
                        self.model += self.t[j] >= self.t[i] + self.cost_matrix[i][j] - (1 - self.x[i, j]) * 10000000

        self.solver = pulp.CPLEX_CMD(
            path="/home/yonikid/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux/cplex",
            msg=False,
            options=[
                'set timelimit 1'
            ]
        )

    def __iter__(self):
        return self

    def __next__(self):
        self.model.solve(self.solver)

        if pulp.LpStatus[self.model.status] == 'Infeasible':
            return {"Allocations": {}, "Cost": math.inf}

        current_edges = {(i, j) for (i, j) in self.x if pulp.value(self.x[i, j]) > 0.5}
        service_times = {j: int(pulp.value(self.t[j])) for j in self.goal_indices}

        paths = {}
        for a in range(self.num_agents):
            curr_path = [self.all_nodes[a]]
            current = a
            while True:
                next_node = next((j for (i, j) in current_edges if i == current), None)
                if next_node is None:
                    break
                curr_path.append(self.all_nodes[next_node])
                current = next_node
            paths[a] = curr_path

        # Add exclusion constraint to prevent repeating this edge set
        self.model += pulp.lpSum([self.x[i, j] for (i, j) in current_edges]) <= len(current_edges) - 1

        return {"Allocations": paths, "Cost": sum(service_times.values())}

    def precompute_costs(self, GoalLocations):
        precomputed_cost = defaultdict(lambda: 1000000)

        for goal in GoalLocations:
            self.BFS(goal, precomputed_cost)

        return precomputed_cost

    def BFS(self, goal, precomputed_cost):
        visited = np.zeros(self.MapAndDims["Cols"] * self.MapAndDims["Rows"], dtype=bool)
        queue = deque([(goal, 0)])

        while queue:
            current_loc, cost = queue.popleft()

            if visited[current_loc]:
                continue
            visited[current_loc] = True

            precomputed_cost[(current_loc, goal)] = cost

            for neighbor_loc, new_cost in self.get_neighbors(current_loc, cost):
                if not visited[neighbor_loc]:
                    queue.append((neighbor_loc, new_cost))

    def get_neighbors(self, current_loc, cost):
        neighbors = []

        for neighborLoc in [current_loc + 1, current_loc + self.MapAndDims["Cols"], current_loc - 1,
                            current_loc - self.MapAndDims["Cols"]]:
            if self.validate_move(neighborLoc, current_loc):
                neighbors.append((neighborLoc, cost + 1))

        return neighbors

    def validate_move(self, loc_after_move, loc):
        # Extract the agent's location and direction before taking the next step

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < self.MapAndDims["Cols"] * self.MapAndDims["Rows"]):
            return False

        # If the agent is at the right boundary, it cannot move right
        if loc % self.MapAndDims["Cols"] == self.MapAndDims["Cols"] - 1 and loc_after_move % \
                self.MapAndDims["Cols"] == 0:
            return False

        # If the agent is at the left boundary, it cannot move left
        if loc % self.MapAndDims["Cols"] == 0 and loc_after_move % self.MapAndDims["Cols"] == \
                self.MapAndDims["Cols"] - 1:
            return False

        if self.MapAndDims["Map"][loc_after_move] != 0:
            return False

        return True

    def build_cost_matrix(self):
        cost_matrix = np.full((self.total_nodes, self.total_nodes), np.inf)
        for i, agent in enumerate(self.all_nodes):
            for j, goal in enumerate(self.all_nodes):
                key = (agent, goal)
                cost_matrix[i, j] = self.cost_dict.get(key, np.inf)
        return cost_matrix

#
# d = {"Rows": 11, "Cols": 11, "Map": [0 for _ in range(11 * 11)]}
# p = kBestSequencingByService([0, 10], [110, 120], d)
# print(next(p))
# print(next(p))
# print(next(p))
# print(next(p))
# print(next(p))
# print(next(p))
# print(next(p))