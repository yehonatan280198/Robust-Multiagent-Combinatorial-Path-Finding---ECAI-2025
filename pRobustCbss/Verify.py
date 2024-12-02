import math
import random
from scipy.stats import norm


def run_s_simulations(s0, paths, delaysProb):
    # Counter for successful simulations (no collisions)
    count_success = 0

    # Run s0 simulations
    for sim in range(s0):
        # Create a copy of the paths for independent simulation
        paths_copy = {agent: list(path) for agent, path in paths.items()}
        # Initialize the set of active agents (agents that have not finished their path)
        active_agents = {agent for agent, path in paths_copy.items() if len(path) > 1}
        # Flag to indicate if a collision occurs
        collision = False

        while active_agents:
            # Set to track current agent locations
            locs = set()
            # Set to track agents that have completed their paths
            agents_to_remove = set()

            for agent in list(active_agents):
                # Current path of the agent
                path = paths_copy[agent]

                # Simulate agent movement with a delay probability
                if random.random() > delaysProb[agent]:
                    # Remove the first step if the agent moves
                    path.pop(0)

                # Current location of the agent
                loc = path[0][0]
                # Check for collision
                if loc in locs:
                    collision = True
                    break
                locs.add(loc)

                # If the agent has reached its destination, mark it for removal
                if len(path) == 1:
                    agents_to_remove.add(agent)

            # Stop the simulation if a collision occurs
            if collision:
                break

            # Remove agents that have completed their paths
            active_agents -= agents_to_remove

        # Increment success count if no collision occurred
        if not collision:
            count_success += 1

    # Return the number of successful simulations
    return count_success


def verify(paths, delaysProb, no_collision_prob, verifyAlpha):
    # Calculate initial simulations size (s0) based on the desired confidence level
    z1SubAlphaSquare = (norm.ppf(1 - verifyAlpha)) ** 2
    s0 = max(30, math.ceil(z1SubAlphaSquare * (no_collision_prob / (1 - no_collision_prob))))
    # Initial simulation run
    count_success = run_s_simulations(s0, paths, delaysProb)
    # Additional simulations performed iteratively
    more_simulation = 0

    while True:
        # Calculate the estimated probability of no collision (P0)
        P0 = count_success / (s0 + more_simulation)
        # Upper confidence bound (c1)
        c1 = no_collision_prob + norm.ppf(1 - verifyAlpha) * math.sqrt((no_collision_prob * (1 - no_collision_prob)) / s0)
        # Lower confidence bound (c2)
        c2 = no_collision_prob - norm.ppf(1 - verifyAlpha) * math.sqrt((no_collision_prob * (1 - no_collision_prob)) / s0)

        # If P0 is greater than or equal to the upper bound, the solution is likely p-robust
        if P0 >= c1:
            return True
        # If P0 is less than the lower bound, the solution is not p-robust
        if P0 < c2:
            return False

        # If no decision, add one more simulation
        more_simulation += 1
        count_success += run_s_simulations(more_simulation, paths, delaysProb)
