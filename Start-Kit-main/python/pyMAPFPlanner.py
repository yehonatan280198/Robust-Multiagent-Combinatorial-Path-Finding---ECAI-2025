import MAPF
import sys

sys.path.append('/home/yonikid/Desktop/SimulatorAgents')
from pRobustCbss.Run_pRobustCbss import pRobustCbss

# 0=Action.FW, 1=Action.CR, 2=Action.CCR, 3=Action.W

class pyMAPFPlanner:
    def __init__(self, pyenv=None):
        self.paths = []
        if pyenv is not None:
            self.env = pyenv.env
        print("pyMAPFPlanner created!  python debug")

        self.lastTimeMove = []

    def initialize(self, preprocess_time_limit: int):
        print("planner initialize done... python debug")
        return True

    def plan(self, time_limit):
        return self.sample_priority_planner(time_limit)

    def sample_priority_planner(self, time_limit: int):
        actions = [MAPF.Action.W] * len(self.env.curr_states)
        for i in range(self.env.num_of_agents):
            if not self.env.goal_locations[i]:
                actions[i] = MAPF.Action.NA
            else:
                index = self.paths[i].index((self.env.curr_states[i].location, self.env.curr_states[i].orientation)) + 1
                if self.paths[i][index][0] != self.env.curr_states[i].location:
                    actions[i] = MAPF.Action.FW
                elif self.paths[i][index][1] != self.env.curr_states[i].orientation:
                    incr = self.paths[i][index][1] - self.env.curr_states[i].orientation
                    if incr == 1 or incr == -3:
                        actions[i] = MAPF.Action.CR
                    elif incr == -1 or incr == 3:
                        actions[i] = MAPF.Action.CCR

        return actions

    def updateTasks(self, currentAgents):
        locations = [(self.env.curr_states[agent].location, self.env.curr_states[agent].orientation) for agent in currentAgents]
        taskLocs = [task.location for task in self.env.unfinishedTasks]
        delaysProb = {i: self.env.FailureProbability[agent] for i, agent in enumerate(currentAgents)}
        self.paths = pRobustCbss(locations, taskLocs, self.env.NoCollisionProbability, delaysProb, self.env.cols, self.env.rows, self.env.verifyAlpha,False, True, True).Solution

        goal_locations = [[] for _ in range(len(currentAgents))]
        for agent, path in self.paths.items():
            loc_path, _ = zip(*path)
            for loc in loc_path:
                if loc in taskLocs and all(loc != item[0] for item in goal_locations[currentAgents[agent]]):
                    task = self.env.unfinishedTasks[taskLocs.index(loc)]
                    goal_locations[currentAgents[agent]].append((task.location, self.env.curr_timestep, task.task_id))
                    task.t_assigned = self.env.curr_timestep
                    task.agent_assigned = currentAgents[agent]

        self.env.goal_locations = goal_locations


if __name__ == "__main__":
    test_planner = pyMAPFPlanner()
    test_planner.initialize(100)
