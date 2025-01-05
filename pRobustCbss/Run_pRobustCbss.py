import copy
import math
from itertools import combinations
from queue import PriorityQueue
import heapq

from pRobustCbss.LowLevelPlan import LowLevelPlan
from pRobustCbss.NodeStateConstClasses import Node, negConst, posConst
from pRobustCbss.Verify import verify, verifyOriginal
from pRobustCbss.kBestSequencing import kBestSequencing
from pRobustCbss.kBestSequencingWithGLKH import kBestSequencingWithGLKH


class pRobustCbss:

    def __init__(self, Positions, GoalLocations, no_collision_prob, delaysProb, dict_of_map_and_dim, verifyAlpha):
        self.Positions = Positions  # Initial positions of agents
        self.GoalLocations = GoalLocations  # Locations of goals
        self.no_collision_prob = no_collision_prob  # The Probability of no collision
        self.delaysProb = delaysProb  # Delay probabilities for each agent
        self.MapAndDims = dict_of_map_and_dim
        self.verifyAlpha = verifyAlpha

        self.OPEN = PriorityQueue()  # Open list for CBS nodes, prioritized by cost
        self.num_roots_generated = 0  # Counter for the number of root nodes generated
        self.kOptimalSequences = {}  # Dictionary to store k-optimal sequences of allocations
        self.kBestSequencingWithGLKH = kBestSequencingWithGLKH(self.Positions, self.GoalLocations, self.MapAndDims)

        self.Solution = self.run()

    def run(self):

        # Calculate the best sequence of task allocations (k=1)
        self.kOptimalSequences[1] = self.kBestSequencingWithGLKH.run(1)
        # Increment root node counter
        self.num_roots_generated += 1

        # Create the root node
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.kOptimalSequences[1]
        # Generate paths and calculate the cost for the root node
        LowLevelPlan(Root, self.MapAndDims, self.Positions, list(range(len(self.Positions))))
        # Add the root node to the open list
        self.OPEN.put((Root.g, Root))

        # Continue processing nodes in the open list until it is empty
        while not self.OPEN.empty():

            # Get the node with the lowest cost
            _, N = self.OPEN.get()
            # Check if a new root needs to be generated
            N = self.CheckNewRoot(N)

            if N is None:
                continue

            # If the paths in the current node are verified as valid, avoiding collisions with probability P, return them as the solution
            if verify(N.paths, self.delaysProb, self.no_collision_prob, self.verifyAlpha):
                return N.paths

            # Identify the first conflict in the paths
            _, _, x, agent1AndTime, agent2AndTime = self.getlConflict(N)

            # Generate child nodes with constraints to resolve the conflict
            A1 = self.GenChild(N, negConst(agent1AndTime[0], x, agent1AndTime[1]))
            A2 = self.GenChild(N, negConst(agent2AndTime[0], x, agent2AndTime[1]))
            A3 = self.GenChild(N, posConst(agent1AndTime[0], agent2AndTime[0], x, agent1AndTime[1], agent2AndTime[1]))

            # Add child nodes to the open list
            self.OPEN.put((A1.g, A1))
            self.OPEN.put((A2.g, A2))
            self.OPEN.put((A3.g, A3))

    def CheckNewRoot(self, N):
        # If the current node cost is within the threshold of the current optimal sequence
        if N.g <= self.kOptimalSequences[self.num_roots_generated]["Cost"]:
            return N

        # Generate a new root with an updated sequence
        self.num_roots_generated += 1
        self.kOptimalSequences[self.num_roots_generated] = self.kBestSequencingWithGLKH.run(self.num_roots_generated)

        if self.kOptimalSequences[self.num_roots_generated]["Cost"] == math.inf:
            return N

        # Create a new root node
        newRoot = Node()
        newRoot.sequence = self.kOptimalSequences[self.num_roots_generated]
        # Calculate paths and cost for the new root
        LowLevelPlan(newRoot, self.MapAndDims, self.Positions, list(range(len(self.Positions))))

        self.OPEN.put((newRoot.g, newRoot))
        self.OPEN.put((N.g, N))
        return None

    # def CheckNewRootOriginal(self, N):
    #     # If the current node cost is within the threshold of the current optimal sequence
    #     if N.g <= self.kOptimalSequences[self.num_roots_generated]["Cost"]:
    #         return N
    #
    #     # Generate a new root with an updated sequence
    #     self.num_roots_generated += 1
    #     # self.kOptimalSequences[self.num_roots_generated] = kBestSequencing(self.Positions, self.GoalLocations,
    #     #                                                                    self.num_roots_generated,
    #     #                                                                    self.num_of_cols, self.rotate).Solution
    #
    #     self.kOptimalSequences[self.num_roots_generated] = kBestSequencingWithGLKH(self.Positions, self.GoalLocations, self.num_roots_generated, self.num_of_cols).Solution
    #
    #     # Create a new root node
    #     newRoot = Node()
    #     newRoot.sequence = self.kOptimalSequences[self.num_roots_generated]
    #     # Calculate paths and cost for the new root
    #     LowLevelPlan(newRoot, self.num_of_cols, self.num_of_rows, self.Positions, list(range(len(self.Positions))))
    #
    #     if N.g <= newRoot.g:
    #         self.OPEN.put((newRoot.g, newRoot))
    #         return N
    #
    #     self.OPEN.put((N.g, N))
    #     return newRoot

    def getlConflict(self, N):
        heap = []
        # Iterate over unique pairs of agents
        for agent1, agent2 in combinations(N.paths.keys(), 2):
            path1 = N.paths[agent1]
            path2 = N.paths[agent2]

            # Create loc-time dictionaries
            locTimes1 = {}
            for i, (loc, _) in enumerate(path1):
                if loc not in locTimes1:
                    locTimes1[loc] = i

            locTimes2 = {}
            for i, (loc, _) in enumerate(path2):
                if loc not in locTimes2:
                    locTimes2[loc] = i

            # Detect location conflicts
            common_locs = locTimes1.keys() & locTimes2.keys()
            for loc in common_locs:
                time1 = locTimes1[loc]
                time2 = locTimes2[loc]
                delta = abs(time1 - time2)
                Time = min(time1, time2)

                if time1 <= time2:
                    heapq.heappush(heap, (delta, Time, loc, (agent1, Time), (agent2, Time + delta)))
                else:
                    heapq.heappush(heap, (delta, Time, loc, (agent1, Time + delta), (agent2, Time)))

            # Create edge-time dictionaries
            edgeTimes1 = {}
            for i in range(len(path1) - 1):
                edge = (path1[i][0], path1[i + 1][0])
                if edge not in edgeTimes1:
                    edgeTimes1[edge] = i + 1

            edgeTimes2 = {}
            for i in range(len(path2) - 1):
                edge = (path2[i][0], path2[i + 1][0])
                if edge not in edgeTimes2:
                    edgeTimes2[edge] = i + 1

            # Detect edge conflicts, including reversed edges
            for edge1, time1 in edgeTimes1.items():
                reversed_edge1 = (edge1[1], edge1[0])
                if reversed_edge1 in edgeTimes2:
                    time2 = edgeTimes2.get(reversed_edge1)
                    delta = abs(time1 - time2)
                    Time = min(time1, time2)

                    if time1 <= time2:
                        heapq.heappush(heap, (delta, Time, edge1, (agent1, Time), (agent2, Time + delta)))
                    else:
                        heapq.heappush(heap, (delta, Time, edge1, (agent1, Time + delta), (agent2, Time)))

        # Return the first conflict in the heap
        return heapq.heappop(heap)

    def GenChild(self, N, NewCons):
        A = Node()
        A.constraint = copy.deepcopy(N.constraint)
        A.paths = {agent: path[:] for agent, path in N.paths.items()}
        A.sequence = N.sequence
        A.g = N.g

        if isinstance(NewCons, negConst):
            A.constraint[NewCons.agent] = A.constraint[NewCons.agent] | {NewCons}
            LowLevelPlan(A, self.MapAndDims, self.Positions, [NewCons.agent])

        elif isinstance(NewCons, posConst):
            A.constraint[NewCons.agent1] = A.constraint[NewCons.agent1] | {NewCons}
            A.constraint[NewCons.agent2] = A.constraint[NewCons.agent2] | {NewCons}

        return A

    #
    # def getlConflictOriginal(self, N):
    #     # Iterate over unique pairs of agents
    #     for agent1, agent2 in combinations(N.paths.keys(), 2):
    #         path1 = N.paths[agent1]
    #         path2 = N.paths[agent2]
    #
    #         # Create loc-time dictionaries
    #         locTimes1 = {}
    #         for i, (loc, _) in enumerate(path1):
    #             if loc not in locTimes1:
    #                 locTimes1[(i, loc)] =(i, loc)
    #
    #         locTimes2 = {}
    #         for i, (loc, _) in enumerate(path2):
    #             if loc not in locTimes2:
    #                 locTimes2[(i, loc)] = (i, loc)
    #
    #         # Detect location conflicts
    #         common_locs = locTimes1.keys() & locTimes2.keys()
    #         for time, loc in common_locs:
    #             return time, loc, agent1, agent2
    #
    #         # Create edge-time dictionaries
    #         edgeTimes1 = {}
    #         for i in range(len(path1) - 1):
    #             edge = (path1[i][0], path1[i + 1][0])
    #             if edge not in edgeTimes1:
    #                 edgeTimes1[(i + 1, edge)] = (i + 1, edge)
    #
    #         edgeTimes2 = {}
    #         for i in range(len(path2) - 1):
    #             edge = (path2[i][0], path2[i + 1][0])
    #             if edge not in edgeTimes2:
    #                 edgeTimes2[(i + 1, edge)] = (i + 1, edge)
    #
    #         # Detect edge conflicts, including reversed edges
    #         for time1, edge1 in edgeTimes1.values():
    #             reversed_edge1 = (edge1[1], edge1[0])
    #             if (time1, reversed_edge1) in edgeTimes2:
    #                 return time1, (edge1, reversed_edge1), agent1, agent2


