
class Run_Simulation:

    def __init__(self, plan, delaysProb, AgentLocations, GoalLocations, randGen):
        self.plan = plan
        self.delaysProb = delaysProb
        self.AgentLocations = AgentLocations
        self.remainGoals = GoalLocations
        self.randGen = randGen
        self.SOC = 0

    def runSimulation(self):
        # Initialize the set of active agents (agents that have not finished their path)
        active_agents = {agent for agent, path in self.plan.items() if len(path) > 1}
        # Flag to indicate if a collision occurs
        collision = False

        while active_agents:
            new_locs = []
            # Set to track current agent locations
            locsAndEdge = set()
            # Set to track agents that have completed their paths
            finish_agents = set()

            for agent, path in self.plan.items():
                # Current path of the agent
                lastLoc = path[0]

                # Simulate agent movement with a delay probability
                if len(path) != 1 and self.randGen.random() > self.delaysProb[agent]:
                    # Remove the first step if the agent moves
                    path.pop(0)

                new_locs.append(path[0])

                # Current location of the agent
                loc = path[0]
                # Check for collision
                if loc in locsAndEdge or (loc, lastLoc) in locsAndEdge:
                    collision = True
                    break
                locsAndEdge.add(loc)
                locsAndEdge.add((lastLoc, loc))

                # If the agent has reached its destination, mark it for removal
                if len(path) == 1:
                    finish_agents.add(agent)

            # Stop the simulation if a collision occurs
            if collision:
                return False

            self.SOC += len(active_agents)
            # Remove agents that have completed their paths
            active_agents -= finish_agents

            self.AgentLocations = new_locs
            self.remainGoals = list(set(self.remainGoals) - set(new_locs))

        return True
