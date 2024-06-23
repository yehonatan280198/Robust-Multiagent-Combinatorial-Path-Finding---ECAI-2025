import copy
import math
import itertools


def replanning(agents, goals):
    pass


def Calc_Finish_Time_After_Replanning(remain_set, goals):
    replanning(remain_set, goals)

    max_time_to_finish = -math.inf
    for agent in remain_set:
        max_time_to_finish = max(max_time_to_finish, len(agent.get_planned_path()))

    return max_time_to_finish


def Find_Who_To_Repair(agents, goals):
    who_to_repair = set()
    min_time_to_finish = math.inf

    combinations_generator = (itertools.combinations(agents, r) for r in range(len(agents) + 1))
    all_subsets = itertools.chain.from_iterable(combinations_generator)

    for subset in all_subsets:
        remain_set = set(agents) - set(subset)
        finish_time_without_subset = Calc_Finish_Time_After_Replanning(remain_set, goals)
        if finish_time_without_subset < min_time_to_finish:
            min_time_to_finish = finish_time_without_subset
            who_to_repair = set(subset)

    return who_to_repair
