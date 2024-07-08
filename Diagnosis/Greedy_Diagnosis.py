import copy
from Diagnosis.Abstract_Diagnosis import Abstract_Diagnosis


class Greedy_Diagnosis(Abstract_Diagnosis):

    def Find_Who_To_Repair_And_The_Remain_Agents(self, agents, goals):

        min_time_to_finish = self.Calc_Finish_Time_After_Replanning(agents, goals)

        remain_agents = agents.copy()
        who_to_repair = set()
        potential_remain_agents = agents

        improvement = True
        while improvement:
            improvement = False
            best_agent_to_remove = None
            best_time_to_finish = min_time_to_finish

            for agent in remain_agents:
                remain_agents_without_one = copy.deepcopy(set(remain_agents) - {agent})
                time_to_finish = self.Calc_Finish_Time_After_Replanning(remain_agents_without_one, goals)

                if time_to_finish <= best_time_to_finish:
                    best_time_to_finish = time_to_finish
                    best_agent_to_remove = agent
                    potential_remain_agents = remain_agents_without_one

            if best_agent_to_remove is not None:
                remain_agents.remove(best_agent_to_remove)
                who_to_repair.add(best_agent_to_remove)
                min_time_to_finish = best_time_to_finish
                improvement = True

        return list(potential_remain_agents), list(who_to_repair)