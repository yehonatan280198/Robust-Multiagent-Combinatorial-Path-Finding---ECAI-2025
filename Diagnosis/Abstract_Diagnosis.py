import math
from abc import ABC, abstractmethod

from Replanning.Distance_To_Goals_Planning import Distance_To_Goals_Planning
from Replanning.Simple_Prioritized_Planning import Simple_Prioritized_Planning


class Abstract_Diagnosis(ABC):
    def __init__(self, replanningAlgName):
        if replanningAlgName == "Simple_Prioritized":
            self.replanningAlg = Simple_Prioritized_Planning()
        elif replanningAlgName == "Distance_To_Goals_Planning":
            self.replanningAlg = Distance_To_Goals_Planning()

    @abstractmethod
    def Find_Who_To_Repair_And_The_Remain_Agents(self, agents, goals):
        pass

    def Calc_Finish_Time_After_Replanning(self, potential_remain_agents, goals):
        self.replanningAlg.replanning(potential_remain_agents, goals)

        max_time_to_finish = -math.inf
        for agent in potential_remain_agents:
            delay_by_observe = agent.get_delay_by_observations()
            len_of_path = len(agent.get_planned_path())
            max_time_to_finish = max(max_time_to_finish, delay_by_observe * len_of_path)

        return max_time_to_finish

