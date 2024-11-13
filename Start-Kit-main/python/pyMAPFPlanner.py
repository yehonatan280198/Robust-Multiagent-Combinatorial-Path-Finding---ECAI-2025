import MAPF
from typing import Tuple, Set
from queue import PriorityQueue

import sys

from pRobustCbss.Run_pRobustCbss import pRobustCbss

sys.path.append('/home/yonikid/Desktop/SimulatorAgents')


# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:
    def __init__(self, pyenv=None):
        self.paths = []
        self.index = 0
        if pyenv is not None:
            self.env = pyenv.env
        print("pyMAPFPlanner created!  python debug")

        self.lastTimeMove = []

    def initialize(self, preprocess_time_limit: int):
        print("planner initialize done... python debug")
        return True

    def plan(self, time_limit):
        pass
        # self.cbss_planner()
        return self.sample_priority_planner(time_limit)

    def sample_priority_planner(self, time_limit: int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        for i in range(self.env.num_of_agents):
            if not self.env.goal_locations[i]:
                actions[i] = MAPF.Action.NA
            else:
                if self.paths[i][self.index][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif self.paths[i][self.index][1] != self.env.curr_states[i].orientation:
                    incr = self.paths[i][self.index][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

        self.index += 1
        return actions

    def updateTasks(self, currentAgents):
        locations = [(self.env.curr_states[agent].location, self.env.curr_states[agent].orientation) for agent in currentAgents]
        taskLocs = [task.location for task in self.env.unfinishedTasks]
        delaysProb = {i: self.env.observationDelay_TotalMoves[agent][0] for i, agent in enumerate(currentAgents)}
        dms = pRobustCbss(locations, taskLocs, 0.1, delaysProb, self.env.cols)

        # ms = MsStar(locations, taskLocs, self.env.cols, self.env.rows)
        # self.paths = ms.run()
        # print(self.paths)
        #
        # goal_locations = [[] for _ in range(5)]
        # for agent, path in enumerate(self.paths):
        #     loc_path, _ = zip(*path)
        #     for loc in loc_path:
        #         if loc in taskLocs:
        #             task = self.env.unfinishedTasks[taskLocs.index(loc)]
        #             goal_locations[currentAgents[agent]].append((task.location, self.env.curr_timestep, task.task_id))
        #             task.t_assigned = self.env.curr_timestep
        #             task.agent_assigned = currentAgents[agent]
        #
        # self.env.goal_locations = goal_locations


if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
