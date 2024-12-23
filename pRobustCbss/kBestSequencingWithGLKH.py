import math
import os
import subprocess

import numpy as np
from queue import PriorityQueue


def generateMtspPar():
    lines = ["PROBLEM_FILE = files/Mtsp.gtsp\n", "MOVE_TYPE = 5\n", "PATCHING_C = 3\n",
             "PATCHING_A = 2\n", "RUNS = 10\n", "OUTPUT_TOUR_FILE = files/Mtsp.tour\n"]

    with open("files/Mtsp.par", mode="w+") as fpar:
        fpar.writelines(lines)


class kBestSequencingWithGLKH:
    def __init__(self, Positions, GoalLocations, k, num_of_cols):
        self.Positions = Positions
        self.PosToLocDict = {pos: pos[0] for pos in self.Positions}
        self.GoalLocations = GoalLocations                                  # Locations of goals
        self.AllPosAndArtGoals = self.Positions + self.createCopyOfGoals()

        self.k = k                                                          # Number of optimal solutions to find
        self.num_of_cols = num_of_cols                                      # Number of columns in the grid

        self.OPEN = PriorityQueue()                                         # Priority queue for exploring k-best solutions
        self.includedEdgesRealCost = set()                                  # Stores real costs for included edges

        # Precompute Manhattan distances between all points
        self.precomputed_distances = self.precompute_all_costs()

        # Run the algorithm
        self.Solution = self.run()

    def run(self):
        current_dir = os.getcwd()
        os.chdir('/home/yonikid/Desktop/SimulatorAgents/pRobustCbss')

        # Initialize sets for included and excluded edges
        includeE, excludeE = set(), set()
        # Solve the problem for the initial setup
        optimalSequences = self.solveRtsp(includeE, excludeE)
        # Add to queue
        self.OPEN.put((optimalSequences["Cost"], (includeE, excludeE, optimalSequences)))
        # List to store k-best solutions
        S = []

        while not self.OPEN.empty():
            # Pop the best sequence
            _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
            # Append the optimal sequence to results
            S.append(optimalSequences)

            # Stop if k solutions are found
            if len(S) == self.k:
                os.chdir(current_dir)
                return S[self.k - 1]

            # Generate new potential solutions by varying include/exclude sets
            indexEdges = optimalSequences["tour"]
            for index, edge in enumerate(indexEdges):
                self.includedEdgesRealCost = set()
                # Add edges to include set
                newIncludeE = includeE | set(indexEdges[:index])
                for CurrEdge in newIncludeE:
                    self.includedEdgesRealCost.add(CurrEdge)
                # Add the current edge to exclude set
                newExcludeE = excludeE | {indexEdges[index]}
                # Solve again
                PotentialOptimalSequences = self.solveRtsp(newIncludeE, newExcludeE)

                # Validate the new solution by ensuring it respects include/exclude constraints
                if not all(edge in PotentialOptimalSequences["tour"] for edge in newIncludeE):
                    continue

                # Validate the new solution by ensuring it respects include/exclude constraints
                if any(edge in PotentialOptimalSequences["tour"] for edge in newExcludeE):
                    continue

                # Add the valid solution to the queue
                self.OPEN.put((PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

        return {"Allocations": {}, "tour": [], "Cost": math.inf}

    def createCopyOfGoals(self):
        listOfTuplesOfGoals = []
        for goal in self.GoalLocations:
            for direct in range(4):
                self.PosToLocDict[(goal, direct)] = goal
                listOfTuplesOfGoals.append((goal, direct))

        return listOfTuplesOfGoals

    def calculate_heuristic_value(self, pos1, pos2):
        loc1, direct1 = pos1
        loc2, direct2 = pos2

        # loc1 divided by num_of_cols gives row1, remainder gives col1
        row1, col1 = divmod(loc1, self.num_of_cols)
        # loc2 divided by num_of_cols gives row2, remainder gives col2
        row2, col2 = divmod(loc2, self.num_of_cols)
        # Compute Manhattan distance
        ManhattanDistance = abs(row1 - row2) + abs(col1 - col2)

        # Determines vertical movement:
        up_or_down = 3 if row1 > row2 else (1 if row1 < row2 else -1)
        # Determines horizontal movement:
        left_or_right = 2 if col1 > col2 else (0 if col1 < col2 else -1)

        # Check if the agent is facing the direction of movement and ensure the agent is not already in the same row or column as the goal
        if (direct1 == up_or_down or direct1 == left_or_right) and up_or_down != -1 and left_or_right != -1:
            agentDirectionAtGoal = left_or_right if direct1 == up_or_down else up_or_down
            # Calculate the number of turns needed while on the target to face its required direction
            countRotateOnTheGOal = abs(direct2 - agentDirectionAtGoal) if abs(direct2 - agentDirectionAtGoal) != 3 else 1
            return ManhattanDistance + 1 + countRotateOnTheGOal

        # Check if the agent is already in the same row or column as the goal
        elif up_or_down == -1 or left_or_right == -1:
            # Check which direction is left to go
            agentDirectionAtGoal = max(up_or_down, left_or_right)
            # Determine how many turns are needed to face the goal direction
            countRotateToReachTheGoal = abs(direct1 - agentDirectionAtGoal) if abs(direct1 - agentDirectionAtGoal) != 3 else 1
            # Calculate the number of turns needed while on the goal to face its required direction
            countRotateOnTheGOal = abs(direct2 - agentDirectionAtGoal) if abs(direct2 - agentDirectionAtGoal) != 3 else 1
            return ManhattanDistance + countRotateToReachTheGoal + countRotateOnTheGOal

        # The agent is not facing the direction of movement and the agent is not already in the same row or column as the goal
        else:
            finalTargetDirection = left_or_right if (direct1 - 1) % 4 == up_or_down or (direct1 + 1) % 4 == up_or_down else up_or_down
            # Calculate the number of turns needed while on the target to face its required direction
            countRotateOnTheGOal = abs(direct2 - finalTargetDirection) if abs(direct2 - finalTargetDirection) != 3 else 1
            return ManhattanDistance + 2 + countRotateOnTheGOal

    def solveRtsp(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.createCostMatrix(includeE, excludeE)
        # Generate LKH parameter file
        generateMtspPar()
        # Generate the MTSP input file
        self.generateMtspFile(costMatrix)
        # Run the LKH solver and return the result
        return self.invoke_lkh()

    def createCostMatrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllPosAndArtGoals), len(self.AllPosAndArtGoals)))
        for row, rowPos in enumerate(self.AllPosAndArtGoals):
            for col, colPos in enumerate(self.AllPosAndArtGoals):

                RealEdge = (self.PosToLocDict[rowPos], self.PosToLocDict[colPos])
                # if not same location
                if rowPos[0] != colPos[0]:
                    cmat[row, col] = 0 if RealEdge in includeE else 99999 if RealEdge in excludeE else self.precomputed_distances[(rowPos, colPos)]

        return cmat

    def generateMtspFile(self, costMatrix):
        nx, ny = costMatrix.shape
        totalSets = len(self.Positions) + len(self.GoalLocations)
        with open("files/Mtsp.gtsp", mode="w+") as ftsp:
            ftsp.writelines(["NAME : mtspf\n", "TYPE : AGTSP\n"])
            ftsp.write("DIMENSION : " + str(nx) + "\n")
            ftsp.write("GTSP_SETS : " + str(totalSets) + "\n")
            ftsp.writelines(
                ["EDGE_WEIGHT_TYPE : EXPLICIT\n", "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n", "EDGE_WEIGHT_SECTION\n"])
            for ix in range(nx):
                nline = ""
                for iy in range(nx):
                    nline = nline + str(int(costMatrix[(ix, iy)])) + " "
                ftsp.write(nline + "\n")

            ftsp.write("GTSP_SET_SECTION\n")
            for agent in range(1, len(self.Positions)+1):
                ftsp.write(f"{agent} {agent} -1\n")

            currSet = len(self.Positions) + 1
            index = len(self.Positions) + 1
            for goal in range(len(self.GoalLocations)):
                ftsp.write(f"{currSet} {index} {index+1} {index+2} {index+3} -1\n")
                currSet += 1
                index += 4
            ftsp.close()

    def invoke_lkh(self):
        cmd = ["/home/yonikid/Desktop/SimulatorAgents/pRobustCbss/GLKH-1.1/GLKH", "files/Mtsp.par"]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

        mtsp_tours = {"Allocations": {}, "tour": {}}

        with open("files/Mtsp.tour", mode="r") as fres:
            lines = fres.readlines()
            mtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6                                                          # Starting index of the tour in the output file
            val = int(lines[ix])
            currAgentTour = []
            tour = []
            agent = -1
            first = True

            # Read until the end of the tour
            while val != -1:
                goal = self.AllPosAndArtGoals[val - 1]
                tour.append(goal)
                if first:
                    agent = val - 1
                    currAgentTour.append(self.PosToLocDict[goal])
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.Positions):
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [self.PosToLocDict[goal]]
                    agent = val - 1
                else:
                    currAgentTour.append(self.PosToLocDict[goal])

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour
            # Create tour as pairs of locations
            mtsp_tours["tour"] = {(self.PosToLocDict[tour[i]], self.PosToLocDict[tour[i + 1]]): self.precomputed_distances[(tour[i], tour[i+1])] for i in range(len(tour) - 1)}

            # Adjust cost for included edges
            for loc in list(mtsp_tours["tour"].keys()):
                if loc in self.includedEdgesRealCost:
                    mtsp_tours["Cost"] += mtsp_tours["tour"][loc]

            mtsp_tours["tour"] = list(mtsp_tours["tour"].keys())

        return mtsp_tours

    def precompute_all_costs(self):
        precomputed_distances = {}
        for i, pos1 in enumerate(self.AllPosAndArtGoals):
            for j, pos2 in enumerate(self.AllPosAndArtGoals):

                if pos1[0] == pos2[0]:
                    precomputed_distances[(pos1, pos2)] = 0

                # If it's the next initial position, assign zero cost
                if i < len(self.Positions) and len(self.Positions) > j == (i + 1) % len(self.Positions):
                    precomputed_distances[(pos1, pos2)] = 0

                # If it's not the next initial position, assign a high cost
                elif i < len(self.Positions) and len(self.Positions) > j != (i + 1) % len(self.Positions):
                    precomputed_distances[(pos1, pos2)] = 99999

                # If the edge goes from a goal to an initial position, assign zero cost
                elif i >= len(self.Positions) > j:
                    precomputed_distances[(pos1, pos2)] = 0
                else:
                    precomputed_distances[(pos1, pos2)] = self.calculate_heuristic_value(pos1, pos2)

        return precomputed_distances


# p = kBestSequencingWithGLKH([(5, 1), (31, 2), (51, 0)], [29, 53], 3, 12).Solution

# p = kBestSequencingWithGLKH([(50,0), (89,3)], [17, 56], 1, 12).Solution

# print(p)



