import copy
from collections import defaultdict
from Replanning.Abstract_Replanning import Abstract_Replanning


class Simple_Prioritized_Planning(Abstract_Replanning):

    def replanning(self, potential_remain_agents, goals):
        sorted_potential_remain_agents = sorted(potential_remain_agents, key=lambda agent: agent.get_id())
        loc_of_agent = {agent.get_id(): agent.get_location() for agent in sorted_potential_remain_agents}
        remain_agents_binds_goals = defaultdict(list)
        copies_goals = copy.deepcopy(goals)
        potential_goal = None

        while len(copies_goals) != 0:

            for agent in sorted_potential_remain_agents:
                min_distance = float('inf')
                ID = agent.get_id()

                for goal in copies_goals:
                    distance = abs(loc_of_agent[ID][0] - goal[0]) + abs(loc_of_agent[ID][1] - goal[1])

                    if distance < min_distance:
                        min_distance = distance
                        potential_goal = goal

                remain_agents_binds_goals[agent.get_id()].append(potential_goal)
                copies_goals.remove(potential_goal)
                loc_of_agent[ID] = potential_goal

                if len(copies_goals) == 0:
                    break

        self.find_path_foreach_remain_agent(sorted_potential_remain_agents, remain_agents_binds_goals)
