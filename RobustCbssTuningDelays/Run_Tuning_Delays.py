from collections import defaultdict
from queue import PriorityQueue
import math

from RobustCbssTuningDelays.FindConflict import FindConflict
from RobustCbssTuningDelays.LowLevelPlan import LowLevelPlan
from RobustCbssTuningDelays.NodeStateClasses import Node
from RobustCbssTuningDelays.Verify import Verify
from RobustCbssTuningDelays.kBestSequencingByService import kBestSequencingByService


class RobustCbssTuningDelays:
    def __init__(self, AgentLocations, GoalLocations, no_collision_prob, delaysProb, MapAndDims, verifyAlpha):
        self.AgentLocations = AgentLocations  # Locations of Agents
        self.GoalLocations = GoalLocations  # Locations of goals

        self.ResolvedConflicts = 0

        self.OPEN = PriorityQueue()  # Open list for CBS nodes, prioritized by cost
        self.Num_roots_generated = 0  # Counter for the number of root nodes generated
        self.K_optimal_sequences = {}  # Dictionary to store k-optimal sequences of allocations

        self.K_Best_Seq_Solver = kBestSequencingByService(self.AgentLocations, self.GoalLocations, MapAndDims)
        self.LowLevelPlanner = LowLevelPlan(MapAndDims, self.AgentLocations, self.K_Best_Seq_Solver.cost_dict)
        self.verify_algorithm = Verify(delaysProb, no_collision_prob, verifyAlpha)
        self.findConflict_algorithm = FindConflict()

        self.Solution = self.run()

    ####################################################### run ############################################################

    def run(self):
        # Calculate the best sequence of task allocations (k=1)
        self.K_optimal_sequences[1] = next(self.K_Best_Seq_Solver)
        # Increment root node counter
        self.Num_roots_generated += 1

        # Create the root node
        Root = Node()
        # Assign the best sequence of task allocations for all agents to the root node
        Root.sequence = self.K_optimal_sequences[1]
        # Generate paths and calculate the cost for the root node
        self.LowLevelPlanner.runLowLevelPlan(Root, list(range(len(self.AgentLocations))))

        # Add the root node to the open list
        self.OPEN.put((Root.g, Root))

        # Continue processing nodes in the open list until it is empty
        while not self.OPEN.empty():

            # Get the node with the lowest cost
            _, N = self.OPEN.get()
            # Check if a new root needs to be generated
            N = self.CheckNewRoot(N)
            if N is None:
                continue

            # If the paths in the current node are verified as valid, avoiding collisions with probability P, return them as the solution
            if (not N.isPositiveNode) and self.verify_algorithm.verify(N.paths):
                return [N.paths, self.Num_roots_generated, self.ResolvedConflicts]

            # Identify the first conflict in the paths
            conflict = self.findConflict_algorithm.findConflict(N)
            if conflict is None:
                continue
            else:
                self.ResolvedConflicts += 1
                _, _, _, x, agent1AndTime, agent2AndTime = conflict

            # Generate child nodes with constraints to resolve the conflict and add child nodes to the open list
            if agent1AndTime[1] != 0:
                A1 = self.GenChild(N, (agent1AndTime[0], x, agent1AndTime[1]))
                if A1 is not None:
                    self.OPEN.put((A1.g, A1))

            if agent2AndTime[1] != 0:
                A2 = self.GenChild(N, (agent2AndTime[0], x, agent2AndTime[1]))
                if A2 is not None:
                    self.OPEN.put((A2.g, A2))

            A3 = self.GenChild(N, (agent1AndTime[0], agent2AndTime[0], x, agent1AndTime[1], agent2AndTime[1]))
            self.OPEN.put((A3.g, A3))

    ####################################################### Check new root ############################################################

    def CheckNewRoot(self, N):
        print(f"{N.g}, {self.K_optimal_sequences[self.Num_roots_generated]['Cost']}")

        # If the current node cost is within the threshold of the current optimal sequence
        if N.g <= self.K_optimal_sequences[self.Num_roots_generated]["Cost"]:
            return N

        # Generate a new root with an updated sequence
        self.Num_roots_generated += 1
        self.K_optimal_sequences[self.Num_roots_generated] = next(self.K_Best_Seq_Solver)

        if self.K_optimal_sequences[self.Num_roots_generated]["Cost"] == math.inf:
            return N

        # Create a new root node
        newRoot = Node()
        newRoot.sequence = self.K_optimal_sequences[self.Num_roots_generated]
        # Calculate paths and cost for the new root
        self.LowLevelPlanner.runLowLevelPlan(newRoot, list(range(len(self.AgentLocations))))

        self.OPEN.put((newRoot.g, newRoot))
        self.OPEN.put((N.g, N))
        return None

    ####################################################### Get conflict ############################################################

    def GenChild(self, N, NewCons):
        A = Node()
        A.negConstraints = defaultdict(set,
                                       {agent: constraints.copy() for agent, constraints in N.negConstraints.items()})
        A.posConstraints = defaultdict(set,
                                       {agent: constraints.copy() for agent, constraints in N.posConstraints.items()})
        A.paths = {
            agent: {"path": list(info["path"]), "cost": info["cost"]}
            for agent, info in N.paths.items()
        }
        A.sequence = N.sequence
        A.g = N.g

        if len(NewCons) == 3:
            agent, _, _ = NewCons
            A.negConstraints[agent].add(NewCons)
            if not self.LowLevelPlanner.runLowLevelPlan(A, [agent]):
                return None

        else:
            A.isPositiveNode = True
            agent1, agent2, _, _, _ = NewCons
            A.posConstraints[agent1].add(NewCons)
            A.posConstraints[agent2].add(NewCons)

        return A

# print([795, 288, 341, 267, 141, 418, 191, 0, 355, 74]  , [931, 683, 559, 864, 430, 364, 652, 837, 454, 854, 174, 576, 790, 633, 597, 511, 783, 333, 123, 130])
# d = {"Rows": 32, "Cols": 32, "Map": [0 for _ in range(32 * 32)]}
# p = RobustCbssTuningDelays([795, 288, 341, 267, 141, 418, 191, 0, 355, 74] , [931, 683, 559, 864, 430, 364, 652, 837, 454, 854, 174, 576, 790, 633, 597, 511, 783, 333, 123, 130], 0.8, {i: 0.1 for i in range(10)}, d, 0.05)
# print(p.Solution)
