import math
import random
from scipy.stats import norm


def run_s_simulations(s0, paths, delaysProb):
    count_success = 0

    for sim in range(s0):
        paths_copy = {agent: list(path) for agent, path in paths.items()}
        active_agents = {agent for agent, path in paths_copy.items() if len(path) > 1}
        collision = False

        while active_agents:
            locs = set()
            agents_to_remove = set()

            for agent in list(active_agents):
                path = paths_copy[agent]

                if random.random() > delaysProb[agent]:
                    path.pop(0)

                loc = path[0][0]
                if loc in locs:
                    collision = True
                    break
                locs.add(loc)

                if len(path) == 1:
                    agents_to_remove.add(agent)

            if collision:
                break

            active_agents -= agents_to_remove

        if not collision:
            count_success += 1

    return count_success


def verify(paths, delaysProb, no_collision_prob):

    s0 = max(30, pow(norm.ppf(1 - 0.05), 2) * (no_collision_prob / (1 - no_collision_prob)))
    more_simulation = 0
    count_success = run_s_simulations(s0, paths, delaysProb)

    while True:
        P0 = count_success / (s0 + more_simulation)
        c1 = no_collision_prob + norm.ppf(1 - 0.05) * math.sqrt((no_collision_prob * (1 - no_collision_prob)) / s0)
        c2 = no_collision_prob - norm.ppf(1 - 0.05) * math.sqrt((no_collision_prob * (1 - no_collision_prob)) / s0)
        if P0 >= c1:
            return True
        if P0 < c2:
            return False

        more_simulation += 1
        count_success += run_s_simulations(more_simulation, paths, delaysProb)
