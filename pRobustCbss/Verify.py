import math
from itertools import combinations

import networkx as nx


def prob_each_agent_experience_most_d_delays(paths, delaysProb, d):
    P_pai_d = 1
    for agent, path in paths.items():
        P_paiI_d = 0
        for r in range(d):
            P_paiI_d += (delaysProb[agent] ** r) * ((1 - delaysProb[agent]) ** len(path)) * (
                math.comb(r + len(path) - 1, r))
        P_pai_d = P_pai_d * P_paiI_d

    return P_pai_d


def find_d_delays_conflict(paths, d):
    d_delays_conflict = set()

    # Iterate over unique pairs of agents
    for agent1, agent2 in combinations(paths.keys(), 2):
        path1 = paths[agent1]
        path2 = paths[agent2]

        # Create vertex-time dictionaries for fast lookup
        locTimes1 = {}
        for i, (loc, _) in enumerate(path1):
            if loc not in locTimes1:
                locTimes1[loc] = i

        locTimes2 = {}
        for i, (loc, _) in enumerate(path2):
            if loc not in locTimes2:
                locTimes2[loc] = i

        # Detect location conflicts
        common_locs = locTimes1.keys() & locTimes2.keys()
        for loc in common_locs:
            time1 = locTimes1[loc]
            time2 = locTimes2[loc]
            delta = abs(time1 - time2)

            if delta > d:
                continue

            d_delays_conflict.add((agent1, agent2))


        # Create edge-time dictionaries for fast lookup
        edgeTimes1 = {}
        for i in range(len(path1) - 1):
            edge = (path1[i][0], path1[i + 1][0])
            if edge not in edgeTimes1:
                edgeTimes1[edge] = i + 1

        edgeTimes2 = {}
        for i in range(len(path2) - 1):
            edge = (path2[i][0], path2[i + 1][0])
            if edge not in edgeTimes2:
                edgeTimes2[edge] = i + 1

        # Detect edge conflicts, including reversed edges
        for edge1, time1 in edgeTimes1.items():
            reversed_edge1 = (edge1[1], edge1[0])
            if reversed_edge1 in edgeTimes2:
                time2 = edgeTimes2.get(reversed_edge1)
                delta = abs(time1 - time2)

                if delta > d:
                    continue

                d_delays_conflict.add((agent1, agent2))

    return d_delays_conflict


def find_groups_of_agents_by_d_delays(d_delays_conflict):

    conflict_graph = nx.Graph()
    for conflict in d_delays_conflict:
        agent1, agent2 = conflict
        conflict_graph.add_edge(agent1, agent2)

    return list(nx.connected_components(conflict_graph))


def calc_no_conflicts_occurring_in_group(group, d):
    pass


def prob_no_conflicts_occurring_provided_each_agent_experience_most_d_delays(paths, delaysProb, d):
    d_delays_conflict = find_d_delays_conflict(paths, d)
    delay_conflict_groups = find_groups_of_agents_by_d_delays(d_delays_conflict)
    P0_pai_d = 1
    for group in delay_conflict_groups:
        P0_pai_d *= calc_no_conflicts_occurring_in_group(group, d)

    return P0_pai_d


def verify(paths, delaysProb, no_collision_prob):
    d = 0
    while True:
        P_pai_d = prob_each_agent_experience_most_d_delays(paths, delaysProb, d)
        P0_pai_d = prob_no_conflicts_occurring_provided_each_agent_experience_most_d_delays(paths, delaysProb, d)
        LB0 = P_pai_d * P0_pai_d
        UB0 = P_pai_d * P0_pai_d + (1 - P_pai_d)
        if LB0 >= no_collision_prob:
            return True
        if UB0 < no_collision_prob:
            return False
        d += 1
