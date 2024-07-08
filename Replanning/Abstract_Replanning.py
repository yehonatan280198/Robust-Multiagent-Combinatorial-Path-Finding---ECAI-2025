from abc import ABC, abstractmethod
import numpy as np


class Abstract_Replanning(ABC):

    @abstractmethod
    def replanning(self, potential_remain_agents, goals):
        pass

    def find_path_foreach_remain_agent(self, sorted_potential_remain_agents, remain_agents_binds_goals):
        for agent in sorted_potential_remain_agents:
            path = []
            x0, y0 = agent.get_location()
            for goal in remain_agents_binds_goals[agent.get_id()]:
                x1, y1 = goal
                while x0 != x1:
                    x0 += np.sign(x1 - x0)
                    path.append([x0, y0])
                while y0 != y1:
                    y0 += np.sign(y1 - y0)
                    path.append([x0, y0])
                x0, y0 = x1, y1

            agent.set_planned_path(path)
