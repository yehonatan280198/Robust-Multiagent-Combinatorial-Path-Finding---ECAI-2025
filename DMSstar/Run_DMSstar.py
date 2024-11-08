from collections import defaultdict
from queue import PriorityQueue

from DMSstar.Label import Label
from DMSstar.MHPP import SolveMHPP


class DMSstar:
    def __init__(self, initial_locations, taskLocs, num_cols, num_rows, delays):
        self.taskLocs = taskLocs
        self.delays = delays
        self.num_cols = num_cols
        self.num_rows = num_rows

        self.OPEN = PriorityQueue()
        self.F = defaultdict(set)

        L0 = Label(initial_locations, [0] * len(taskLocs), [0] * len(initial_locations))
        SolveMHPP(L0, taskLocs, num_cols)
        self.OPEN.put((L0.f_temp, L0))
        self.F[L0.v].add(L0)


    def run(self):
        while not self.OPEN.empty():
            _, l = self.OPEN.get()
            self.TargetSeq(l.parent, l, l.parent.Ic)


