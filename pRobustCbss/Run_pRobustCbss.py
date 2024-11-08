from pRobustCbss.Node import Node
from queue import PriorityQueue

from pRobustCbss.kBestSequencing import kBestSequencing


class pRobustCbss:

    def __init__(self, locations, taskLocs, no_collision_prob):
        self.locations = locations
        self.taskLocs = taskLocs
        self.no_collision_prob = no_collision_prob

        self.OPEN = PriorityQueue()
        self.num_roots_generated = 0
        self.kOptimalSequences = {}

        self.run()

    def run(self):
        self.kOptimalSequences[1] = kBestSequencing(self.locations, self.taskLocs, k=1)
        self.num_roots_generated += 1
        Root = Node()
        Root.sequence = self.kOptimalSequences[1]
        Root.paths, Root.g = self.LowLevelPlan(Root.sequence, Root.constraint)
        self.OPEN.put((Root.g, Root))

        while not self.OPEN.empty():
            _, N = self.OPEN.get()
            N = self.CheckNewRoot(N)
            if self.verify(N.paths):
                return N.paths


    def LowLevelPlan(self, optimal_sequence, constraint):
        return {}, 0

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
        pass





