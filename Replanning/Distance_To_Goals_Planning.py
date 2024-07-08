import math
from collections import defaultdict
from Replanning.Abstract_Replanning import Abstract_Replanning


class Distance_To_Goals_Planning(Abstract_Replanning):

    def replanning(self, potential_remain_agents, goals):
        sorted_potential_remain_agents = sorted(potential_remain_agents, key=lambda agent: agent.get_id())
        loc_of_agent = {agent.get_id(): agent.get_location() for agent in sorted_potential_remain_agents}

        distancesMatrix = [[0 for _ in goals] for _ in sorted_potential_remain_agents]

        for i, agent in enumerate(sorted_potential_remain_agents):
            ID = agent.get_id()
            for j, goal in enumerate(goals):
                distancesMatrix[i][j] = abs(loc_of_agent[ID][0] - goal[0]) + abs(loc_of_agent[ID][1] - goal[1])

        remain_agents_binds_goals = defaultdict(list)

        while any(cell != math.inf for cell in distancesMatrix[0]):

            min_dist = math.inf
            MinAgent = None
            MinGoal = None
            for i, agent in enumerate(sorted_potential_remain_agents):
                for j, goal in enumerate(goals):
                    if distancesMatrix[i][j] < min_dist:
                        min_dist = distancesMatrix[i][j]
                        MinAgent = [agent, i]
                        MinGoal = [goal, j]

            remain_agents_binds_goals[MinAgent[0].get_id()].append(MinGoal[0])
            loc_of_agent[MinAgent[0].get_id()] = MinGoal[0]

            distance_until = distancesMatrix[MinAgent[1]][MinGoal[1]]

            for i, agent in enumerate(sorted_potential_remain_agents):
                ID = agent.get_id()
                for j, goal in enumerate(goals):
                    if j == MinGoal[1]:
                        distancesMatrix[i][j] = math.inf

                    elif i == MinAgent[1] and distancesMatrix[i][j] != math.inf:
                        distancesMatrix[i][j] = distance_until + abs(loc_of_agent[ID][0] - goal[0]) + abs(loc_of_agent[ID][1] - goal[1])

        self.find_path_foreach_remain_agent(sorted_potential_remain_agents, remain_agents_binds_goals)