import math
import random
from itertools import combinations

from scipy.stats import norm

class Verify:

    def __init__(self, delaysProb, no_collision_prob, verifyAlpha):
        self.delaysProb = delaysProb
        self.no_collision_prob = no_collision_prob
        self.verifyAlpha = verifyAlpha
        self.randGen = random.Random(47)

    def verify(self, paths):
        # Calculate initial simulations size (s0) based on the desired confidence level
        z1SubAlphaSquare = (norm.ppf(1 - self.verifyAlpha)) ** 2
        s0 = max(30, math.ceil(z1SubAlphaSquare * (self.no_collision_prob / (1 - self.no_collision_prob))))

        # Initial simulation run
        count_success = self.run_s_simulations(s0, paths)
        # Additional simulations performed iteratively
        more_simulation = 0

        while True:
            # Calculate the estimated probability of no collision (P0)
            P0 = count_success / (s0 + more_simulation)
            # Upper confidence bound (c1)
            c1 = self.no_collision_prob + norm.ppf(1 - self.verifyAlpha) * math.sqrt(
                (self.no_collision_prob * (1 - self.no_collision_prob)) / s0)
            # Lower confidence bound (c2)
            c2 = self.no_collision_prob - norm.ppf(1 - self.verifyAlpha) * math.sqrt(
                (self.no_collision_prob * (1 - self.no_collision_prob)) / s0)

            # If P0 is greater than or equal to the upper bound, the solution is likely p-robust
            if P0 >= c1:
                return True
            # If P0 is less than the lower bound, the solution is not p-robust
            if P0 < c2:
                return False

            # If no decision, add one more simulation
            more_simulation += 1
            count_success += self.run_s_simulations(more_simulation, paths)

    def run_s_simulations(self, s0, paths):
        count_success = 0

        # Run s0 simulations
        for sim in range(s0):
            # Create a copy of the paths for independent simulation
            paths_copy = {
                agent: {"path": list(info["path"]), "cost": info["cost"]}
                for agent, info in paths.items()
            }

            # Initialize the set of active agents (agents that have not finished their path)
            active_agents = {agent for agent, path in paths_copy.items() if len(path) > 1}
            # Flag to indicate if a collision occurs
            collision = False

            while active_agents:
                # Set to track current agent locations
                locsAndEdge = set()
                # Set to track agents that have completed their paths
                finish_agents = set()

                for agent, path in paths_copy.items():
                    # Current path of the agent
                    lastLoc = path["path"][0]

                    # Simulate agent movement with a delay probability
                    if len(path["path"]) != 1 and self.randGen.random() > self.delaysProb[agent]:
                        # Remove the first step if the agent moves
                        path["path"].pop(0)

                    # Current location of the agent
                    loc = path["path"][0]
                    # Check for collision
                    if loc in locsAndEdge or (loc, lastLoc) in locsAndEdge:
                        collision = True
                        break
                    locsAndEdge.add(loc)
                    locsAndEdge.add((lastLoc, loc))

                    # If the agent has reached its destination, mark it for removal
                    if len(path["path"]) == 1:
                        finish_agents.add(agent)

                # Stop the simulation if a collision occurs
                if collision:
                    break

                # Remove agents that have completed their paths
                active_agents -= finish_agents

            # Increment success count if no collision occurred
            if not collision:
                count_success += 1

        # Return the number of successful simulations
        return count_success
