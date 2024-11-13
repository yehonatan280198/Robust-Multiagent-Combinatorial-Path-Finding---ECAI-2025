import math

from pRobustCbss.Node import Node
from queue import PriorityQueue
import heapq

from pRobustCbss.kBestSequencing import kBestSequencing


class pRobustCbss:

    def __init__(self, locations, taskLocs, no_collision_prob, delaysProb, num_of_cols):
        self.locations = locations
        self.taskLocs = taskLocs
        self.no_collision_prob = no_collision_prob
        self.delaysProb = delaysProb
        self.num_of_cols = num_of_cols

        self.OPEN = PriorityQueue()
        self.num_roots_generated = 0
        self.kOptimalSequences = {}

        self.run()

    def run(self):
        self.kOptimalSequences[1] = kBestSequencing(self.locations, self.taskLocs, 1, self.num_of_cols).S
        self.num_roots_generated += 1

        Root = Node()
        Root.sequence = self.kOptimalSequences[1]
        Root.paths, Root.g = self.LowLevelPlan(Root.sequence["Allocations"], Root.constraint)

        self.OPEN.put((Root.g, Root))

        while not self.OPEN.empty():

            _, N = self.OPEN.get()
            N = self.CheckNewRoot(N)

            if self.verify(N.paths):
                return N.paths

            item = self.getlConflict(N)



    def LowLevelPlan(self, Allocations, constraint):
        paths = {}
        for agent, sequence in Allocations.items():
            path = [self.locations[agent]]
            for i in range(len(sequence) - 1):
                curr_loc, curr_direct = list(path[agent][-1])

    def CheckNewRoot(self, N):
        if N.cost <= self.kOptimalSequences[self.num_roots_generated][0]:
            return N

        self.num_roots_generated += 1
        self.kOptimalSequences[self.num_roots_generated] = kBestSequencing(k=self.num_roots_generated)
        newRoot = Node()
        newRoot.sequence = self.kOptimalSequences[self.num_roots_generated]
        newRoot.paths, newRoot.g = self.LowLevelPlan(newRoot.sequence, set())

        if N.cost <= newRoot.g:
            self.OPEN.put((newRoot.g, newRoot))
            return N

        self.OPEN.put((N.g, N))
        return newRoot

        pass

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
        for agent1, path1 in N.paths.items():
            for agent2, path2 in N.paths.items():
                if agent1 == agent2:
                    continue
                locPath1 = [loc for loc, direct in path1]
                locPath2 = [loc for loc, direct in path2]
                common_vals = set(locPath1) & set(locPath2)
                for x in common_vals:
                    time = min(locPath1.index(x), locPath2.index(x))
                    delta = abs(locPath1.index(x) - locPath2.index(x))
                    heapq.heappush(heap, (agent1, agent2, x, time, delta))

        return heapq.heappop(heap)

