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


def create_loc_times(path):
    locTimes = {}
    for i, (loc, _) in enumerate(path):
        locTimes.setdefault(loc, i)
    return locTimes


def create_edge_times(path):
    edgeTimes = {}
    for i in range(len(path) - 1):
        edge = (path[i][0], path[i + 1][0])
        edgeTimes.setdefault(edge, i + 1)
    return edgeTimes


def getConflict(N):
    heap = []
    # Iterate over unique pairs of agents
    for agent1, agent2 in combinations(N.paths.keys(), 2):
        allPosConstDict = {
            (pos_const.x, (pos_const.agent1, pos_const.agent1_time), (pos_const.agent2, pos_const.agent2_time)): True
            for pos_const in N.posConstraints[agent1] | N.posConstraints[agent2]}
        path1, path2 = N.paths[agent1], N.paths[agent2]

        # Create loc-time dictionaries
        locTimes1 = create_loc_times(path1)
        locTimes2 = create_loc_times(path2)

        # Detect location conflicts
        for loc in locTimes1.keys() & locTimes2.keys():
            time1, time2 = locTimes1[loc], locTimes2[loc]
            delta = abs(time1 - time2)
            Time = min(time1, time2)

            agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

            if (loc, (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                heapq.heappush(heap, (delta, Time, loc, (agent1, agent1_time), (agent2, agent2_time)))

        # Create edge-time dictionaries
        edgeTimes1 = create_edge_times(path1)
        edgeTimes2 = create_edge_times(path2)

        # Detect edge conflicts, including reversed edges
        for edge1, time1 in edgeTimes1.items():
            reversed_edge1 = (edge1[1], edge1[0])
            if reversed_edge1 in edgeTimes2:
                time2 = edgeTimes2[reversed_edge1]
                delta = abs(time1 - time2)
                Time = min(time1, time2)

                agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

                if (frozenset(edge1), (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                    heapq.heappush(heap, (delta, Time, frozenset(edge1), (agent1, agent1_time), (agent2, agent2_time)))

    # Return the first conflict in the heap
    if heap:
        return heapq.heappop(heap)
    else:
        return None


class pRobustCbss:

    def __init__(self, Positions, GoalLocations, no_collision_prob, delaysProb, dict_of_map_and_dim, verifyAlpha):
        self.Positions = Positions  # Initial positions of agents
        self.GoalLocations = GoalLocations  # Locations of goals
        self.No_collision_prob = no_collision_prob  # The Probability of no collision
        self.DelaysProb = delaysProb  # Delay probabilities for each agent
        self.MapAndDims = dict_of_map_and_dim
        self.VerifyAlpha = verifyAlpha

        self.OPEN = PriorityQueue()  # Open list for CBS nodes, prioritized by cost
        self.Num_roots_generated = 0  # Counter for the number of root nodes generated
        self.K_optimal_sequences = {}  # Dictionary to store k-optimal sequences of allocations
        self.K_best_sequencing_with_GLKH = kBestSequencingWithGLKH(self.Positions, self.GoalLocations, self.MapAndDims)

        self.Solution = self.run()

    ####################################################### run ############################################################

    def run(self):

        # Calculate the best sequence of task allocations (k=1)
        self.K_optimal_sequences[1] = self.K_best_sequencing_with_GLKH.Find_K_Best_Solution(k=1)
        # Increment root node counter
        self.Num_roots_generated += 1

        # Create the root node
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.K_optimal_sequences[1]
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
            if verify(N.paths, self.DelaysProb, self.No_collision_prob, self.VerifyAlpha):
                return [N.paths, self.K_best_sequencing_with_GLKH.Counter_Solver_Tsp_For_Test,
                        self.K_best_sequencing_with_GLKH.Counter_BFS_For_Test]

            # Identify the first conflict in the paths
            const = getConflict(N)
            if const is None:
                continue
            else:
                _, _, x, agent1AndTime, agent2AndTime = const

            # Generate child nodes with constraints to resolve the conflict and add child nodes to the open list
            if agent1AndTime[1] != 0:
                A1 = self.GenChild(N, negConst(agent1AndTime[0], x, agent1AndTime[1]))
                self.OPEN.put((A1.g, A1))

            if agent2AndTime[1] != 0:
                A2 = self.GenChild(N, negConst(agent2AndTime[0], x, agent2AndTime[1]))
                self.OPEN.put((A2.g, A2))

            A3 = self.GenChild(N, posConst(agent1AndTime[0], agent2AndTime[0], x, agent1AndTime[1], agent2AndTime[1]))
            self.OPEN.put((A3.g, A3))

    ####################################################### Check new root ############################################################

    def CheckNewRoot(self, N):
        # If the current node cost is within the threshold of the current optimal sequence
        if N.g <= self.K_optimal_sequences[self.Num_roots_generated]["Cost"]:
            return N

        # Generate a new root with an updated sequence
        self.Num_roots_generated += 1
        self.K_optimal_sequences[self.Num_roots_generated] = self.K_best_sequencing_with_GLKH.Find_K_Best_Solution(
            k=self.Num_roots_generated)

        if self.K_optimal_sequences[self.Num_roots_generated]["Cost"] == math.inf:
            return N

        # Create a new root node
        newRoot = Node()
        newRoot.sequence = self.K_optimal_sequences[self.Num_roots_generated]
        # Calculate paths and cost for the new root
        LowLevelPlan(newRoot, self.MapAndDims, self.Positions, list(range(len(self.Positions))))

        self.OPEN.put((newRoot.g, newRoot))
        self.OPEN.put((N.g, N))
        return None

    ####################################################### Get conflict ############################################################

    def GenChild(self, N, NewCons):
        A = Node()
        A.negConstraints = copy.deepcopy(N.negConstraints)
        A.posConstraints = copy.deepcopy(N.posConstraints)
        A.paths = {agent: path[:] for agent, path in N.paths.items()}
        A.sequence = N.sequence
        A.g = N.g

        if isinstance(NewCons, negConst):
            A.negConstraints[NewCons.agent] = A.negConstraints[NewCons.agent] | {NewCons}
            LowLevelPlan(A, self.MapAndDims, self.Positions, [NewCons.agent])

        elif isinstance(NewCons, posConst):
            A.posConstraints[NewCons.agent1] = A.posConstraints[NewCons.agent1] | {NewCons}
            A.posConstraints[NewCons.agent2] = A.posConstraints[NewCons.agent2] | {NewCons}

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


# p = pRobustCbss([(848, 1), (941, 0), (39, 2), (152, 2), (953, 3)], [110, 308, 738, 460, 1020, 956, 334, 264, 459, 476, 727, 49, 810, 743, 685, 395, 377, 555, 320, 642], 0.7,
#                 {i: 0.05 for i in range(5)}, {"Rows": 32, "Cols": 32, "Map": [0 for _ in range(32 * 32)]}, 0.05)
#
# print(p.Solution)
