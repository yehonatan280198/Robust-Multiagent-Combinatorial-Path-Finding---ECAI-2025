import math
from itertools import combinations

from pRobustCbss.LowLevelPlan import LowLevelPlan
from pRobustCbss.NodeAndConstClass import Node, negConst, posConst
from queue import PriorityQueue
import heapq

from pRobustCbss.kBestSequencing import kBestSequencing


class pRobustCbss:

    def __init__(self, locations, taskLocs, no_collision_prob, delaysProb, num_of_cols, num_of_rows):
        self.locations = locations
        self.taskLocs = taskLocs
        self.no_collision_prob = no_collision_prob
        self.delaysProb = delaysProb
        self.num_of_cols = num_of_cols
        self.num_of_rows = num_of_rows

        self.OPEN = PriorityQueue()
        self.num_roots_generated = 0
        self.kOptimalSequences = {}

        self.run()

    def run(self):
        # Calculate the best sequence of task allocations for all agents
        self.kOptimalSequences[1] = kBestSequencing(self.locations, self.taskLocs, 1, self.num_of_cols).S
        # Increment the counter for the number of allocation roots generated
        self.num_roots_generated += 1

        # Initialize a root node for the allocation sequence
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.kOptimalSequences[1]
        # Generate paths and compute the cost for the root node based on the allocation sequence and constraints
        Root.paths, Root.g = LowLevelPlan(Root.sequence["Allocations"], Root.constraint, self.num_of_cols, self.num_of_rows, self.locations).solution
        print(Root.paths, Root.g)
        return

        # Add the root node to the open list based on its cost 'g' for exploration
        self.OPEN.put((Root.g, Root))

        # Continue processing nodes in the open list until it is empty
        while not self.OPEN.empty():

            # Retrieve the node with the lowest cost from the open list
            _, N = self.OPEN.get()
            # Check if the current node requires a new root with a different allocation
            N = self.CheckNewRoot(N)

            # If the paths in the current node are verified as valid, avoiding collisions with probability P, return them as the solution
            if self.verify(N.paths):
                return N.paths

            # Identify and unpack conflict details
            delta, time, x, agent1_info, agent2_info = self.getlConflict(N)

            # Generate child nodes with constraints based on the conflict
            A1 = self.GenChild(N, negConst(agent1_info[0], x, agent1_info[1]))
            A2 = self.GenChild(N, negConst(agent2_info[0], x, agent2_info[1]))
            A3 = self.GenChild(N, posConst(agent1_info[0], agent2_info[0], x, agent1_info[1], agent2_info[1]))

            # Add child nodes to the open list for exploration
            self.OPEN.put((A1.g, A1))
            self.OPEN.put((A2.g, A2))
            self.OPEN.put((A3.g, A3))


    def CheckNewRoot(self, N):
        # If the cost of the current node is within the threshold of the current optimal sequence
        if N.cost <= self.kOptimalSequences[self.num_roots_generated]["Cost"]:
            return N

        # Increment the root generation counter and calculate a new optimal sequence
        self.num_roots_generated += 1
        self.kOptimalSequences[self.num_roots_generated] = kBestSequencing(self.locations, self.taskLocs,
                                                                           self.num_roots_generated, self.num_of_cols).S

        # Create a new root node with the updated sequence and calculate paths and cost
        newRoot = Node()
        newRoot.sequence = self.kOptimalSequences[self.num_roots_generated]
        newRoot.paths, newRoot.g = self.LowLevelPlan(newRoot.sequence, set())

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

            # Get the set of locations for each path
            locPath1 = {loc for loc, _ in path1}
            locPath2 = {loc for loc, _ in path2}

            # Find common locations
            common_vals = locPath1 & locPath2

            # Process each common location
            for x in common_vals:
                # Find the minimum time at which the location is visited by either agent
                time1 = next(i for i, (loc, _) in enumerate(path1) if loc == x)
                time2 = next(i for i, (loc, _) in enumerate(path2) if loc == x)
                time = min(time1, time2)
                delta = abs(time1 - time2)

                if time1 <= time2:
                    # Agent1 arrives at the conflict location before or at the same time as agent2
                    heapq.heappush(heap, (delta, time, x, (agent1, time), (agent2, time + delta)))
                else:
                    # Agent2 arrives at the conflict location before agent1
                    heapq.heappush(heap, (delta, time, x, (agent1, time + delta), (agent2, time)))

        # Return the first conflict in the heap
        return heapq.heappop(heap) if heap else None

    def GenChild(self, N, NewCons):
        A = Node()
        A.constraint = N.constraint | set(NewCons)
        A.paths = N.paths
        A.sequence = N.sequence
        A.g = N.g

        if isinstance(NewCons, negConst):
            # A.paths, A.g = self.LowLevelPlan(A.sequence, A.constraint)
            pass
        return A


print(pRobustCbss([(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)], [122, 124, 126, 128, 130], 0.1,
                  {0: 0.1, 1: 0.1, 2: 0.1, 3: 0.1, 4: 0.1}, 12, 12))
