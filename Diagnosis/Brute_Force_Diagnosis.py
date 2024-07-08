import copy
import math
import itertools
from Diagnosis.Abstract_Diagnosis import Abstract_Diagnosis


class Brute_Force_Diagnosis(Abstract_Diagnosis):

    def Find_Who_To_Repair_And_The_Remain_Agents(self, agents, goals):

        who_to_repair, remain_agents = None, None
        min_time_to_finish = math.inf

        combinations_generator = (itertools.combinations(agents, r) for r in range(0, len(agents)))
        all_subsets = itertools.chain.from_iterable(combinations_generator)

        for potential_to_repair in all_subsets:
            potential_remain_agents = copy.deepcopy(set(agents) - set(potential_to_repair))

            finish_time_remain_agents = self.Calc_Finish_Time_After_Replanning(potential_remain_agents, goals)

            if finish_time_remain_agents <= min_time_to_finish:
                min_time_to_finish = finish_time_remain_agents
                remain_agents, who_to_repair = potential_remain_agents, potential_to_repair

        return remain_agents, who_to_repair

