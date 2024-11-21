import math
from itertools import combinations
from queue import PriorityQueue
import heapq

from pRobustCbss.LowLevelPlan import LowLevelPlan
from pRobustCbss.NodeAndConstClass import Node, negConst, posConst
from pRobustCbss.kBestSequencing import kBestSequencing


class pRobustCbss:

    def __init__(self, Positions, GoalLocations, no_collision_prob, delaysProb, num_of_cols, num_of_rows):
        self.Positions = Positions                      # Initial positions of agents
        self.GoalLocations = GoalLocations              # Locations of goals
        self.no_collision_prob = no_collision_prob      # The Probability of no collision
        self.delaysProb = delaysProb                    # Delay probabilities for each agent
        self.num_of_cols = num_of_cols                  # Number of columns in the grid
        self.num_of_rows = num_of_rows                  # Number of rows in the grid

        self.OPEN = PriorityQueue()                     # Open list for CBS nodes, prioritized by cost
        self.num_roots_generated = 0                    # Counter for the number of root nodes generated
        self.kOptimalSequences = {}                     # Dictionary to store k-optimal sequences of allocations

        self.run()

    def run(self):
        # Calculate the best sequence of task allocations (k=1)
        self.kOptimalSequences[1] = kBestSequencing(self.Positions, self.GoalLocations, 1, self.num_of_cols).Solution
        # Increment root node counter
        self.num_roots_generated += 1

        # Create the root node
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.kOptimalSequences[1]
        # Generate paths and calculate the cost for the root node
        Root.paths, Root.g = LowLevelPlan(Root.sequence["Allocations"], Root.constraint, self.num_of_cols, self.num_of_rows, self.Positions).solution
        # Add the root node to the open list
        self.OPEN.put((Root.g, Root))

        # Continue processing nodes in the open list until it is empty
        while not self.OPEN.empty():

            # Get the node with the lowest cost
            _, N = self.OPEN.get()
            # Check if a new root needs to be generated
            N = self.CheckNewRoot(N)

            # If the paths in the current node are verified as valid, avoiding collisions with probability P, return them as the solution
            if self.verify(N.paths):
                return N.paths

            # Identify the first conflict in the paths
            delta, time, x, agent1_info, agent2_info = self.getlConflict(N)

            # Generate child nodes with constraints to resolve the conflict
            A1 = self.GenChild(N, negConst(agent1_info[0], x, agent1_info[1]))
            A2 = self.GenChild(N, negConst(agent2_info[0], x, agent2_info[1]))
            A3 = self.GenChild(N, posConst(agent1_info[0], agent2_info[0], x, agent1_info[1], agent2_info[1]))

            # Add child nodes to the open list
            self.OPEN.put((A1.g, A1))
            self.OPEN.put((A2.g, A2))
            self.OPEN.put((A3.g, A3))

    def CheckNewRoot(self, N):
        # If the current node cost is within the threshold of the current optimal sequence
        if N.cost <= self.kOptimalSequences[self.num_roots_generated]["Cost"]:
            return N

        # Generate a new root with an updated sequence
        self.num_roots_generated += 1
        self.kOptimalSequences[self.num_roots_generated] = kBestSequencing(self.Positions, self.GoalLocations, self.num_roots_generated, self.num_of_cols).Solution

        # Create a new root node
        newRoot = Node()
        newRoot.sequence = self.kOptimalSequences[self.num_roots_generated]
        # Calculate paths and cost for the new root
        newRoot.paths, newRoot.g = LowLevelPlan(newRoot.sequence["Allocations"], set(), self.num_of_cols, self.num_of_rows, self.Positions).solution

        # Add the new root node to OPEN if its cost is lower or equal to the current node's cost
        if N.cost <= newRoot.g:
            self.OPEN.put((newRoot.g, newRoot))
            return N

        # Otherwise, add the current node back to OPEN and return the new root node as the root
        self.OPEN.put((N.g, N))
        return newRoot

    def verify(self, paths):
        d = 0
        while True:
            P_pai_d = self.prob_each_agent_experience_most_d_delays(paths, d)
            P0_pai_d = self.prob_no_conflicts_occuring_provided_each_agent_experience_most_d_delays(paths, d)
            LB0 = P_pai_d * P0_pai_d
            UB0 = P_pai_d * P0_pai_d + (1 - P_pai_d)
            if LB0 >= self.no_collision_prob:
                return True
            if UB0 < self.no_collision_prob:
                return False
            d += 1

        pass

    def prob_each_agent_experience_most_d_delays(self, paths, d):
        P_pai_d = 1
        for agent, path in paths.items():
            P_paiI_d = 0
            for r in range(d):
                P_paiI_d += (self.delaysProb[agent] ** r) * ((1 - self.delaysProb[agent]) ** len(path)) * (
                    math.comb(r + len(path) - 1, r))
            P_pai_d = P_pai_d * P_paiI_d

        return P_pai_d

    def prob_no_conflicts_occuring_provided_each_agent_experience_most_d_delays(self, paths, d):
        return 1

    def getlConflict(self, N):

        heap = []
        # Iterate over unique pairs of agents
        for agent1, agent2 in combinations(N.paths.keys(), 2):
            path1 = N.paths[agent1]
            path2 = N.paths[agent2]

            # Create dictionaries for fast lookup of locations and edges with their times
            locTimes1 = {loc: i for i, (loc, _) in enumerate(path1)}
            locTimes2 = {loc: i for i, (loc, _) in enumerate(path2)}

            # Detect location conflicts
            common_locs = locTimes1.keys() & locTimes2.keys()
            for loc in common_locs:
                time1 = locTimes1[loc]
                time2 = locTimes2[loc]
                delta = abs(time1 - time2)
                time = min(time1, time2)

                if time1 <= time2:
                    heapq.heappush(heap, (delta, time, loc, (agent1, time), (agent2, time + delta)))
                else:
                    heapq.heappush(heap, (delta, time, loc, (agent1, time + delta), (agent2, time)))

            # Create edge dictionaries for fast lookup
            edges1 = {(path1[i][0], path1[i + 1][0]): i for i in range(len(path1) - 1)}
            edges2 = {(path2[i][0], path2[i + 1][0]): i for i in range(len(path2) - 1)}

            # Detect edge conflicts, including reversed edges
            for edge1, time1 in edges1.items():
                reversed_edge1 = (edge1[1], edge1[0])
                if edge1 in edges2 or reversed_edge1 in edges2:
                    time2 = edges2.get(edge1, edges2.get(reversed_edge1))
                    delta = abs(time1 - time2)
                    time = min(time1, time2)
                    heapq.heappush(heap, (delta, time, f"edge {edge1}", (agent1, time), (agent2, time + delta)))

        # Return the first conflict in the heap
        return heapq.heappop(heap) if heap else None

    def GenChild(self, N, NewCons):
        A = Node()
        A.constraint = N.constraint | set(NewCons)      # Add new constraints
        A.paths = N.paths
        A.sequence = N.sequence
        A.g = N.g

        if isinstance(NewCons, negConst):
            # A.paths, A.g = self.LowLevelPlan(A.sequence, A.constraint)
            pass
        return A


pRobustCbss([(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)], [122, 124, 126, 128, 130], 0.1,
                  {0: 0.1, 1: 0.1, 2: 0.1, 3: 0.1, 4: 0.1}, 12, 12)
