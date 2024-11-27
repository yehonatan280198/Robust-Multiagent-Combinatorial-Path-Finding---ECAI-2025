import os
import subprocess

import numpy as np
from queue import PriorityQueue


class kBestSequencing:
    def __init__(self, Positions, GoalLocations, k, num_of_cols):
        self.locations = [loc for loc, direct in Positions]             # Extract only locations, ignoring direction
        self.GoalLocations = GoalLocations                              # Locations of goals
        self.k = k                                                      # Number of optimal solutions to find
        self.num_of_cols = num_of_cols                                  # Number of columns in the grid

        self.OPEN = PriorityQueue()                                     # Priority queue for exploring k-best solutions
        self.includedEdgesRealCost = {}                                 # Stores real costs for included edges

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
                # Add edges to include set
                newIncludeE = includeE | set(indexEdges[:index])
                # Add the current edge to exclude set
                newExcludeE = excludeE | {indexEdges[index]}
                # Solve again
                PotentialOptimalSequences = self.solveRtsp(newIncludeE, newExcludeE)

                # Validate the new solution by ensuring it respects include/exclude constraints
                if (all(edge not in PotentialOptimalSequences["tour"] for edge in newExcludeE) and
                        all(edge in PotentialOptimalSequences["tour"] for edge in newIncludeE)):
                    # Add the valid solution to the queue
                    self.OPEN.put((PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

    def solveRtsp(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.createCostMatrix(includeE, excludeE)
        # Generate LKH parameter file
        self.generateMtspPar()
        # Generate the MTSP input file
        self.generateMtspFile(costMatrix)
        # Run the LKH solver and return the result
        return self.invoke_lkh()

    def createCostMatrix(self, includeE, excludeE):
        # Reset real cost dictionary
        self.includedEdgesRealCost = {}
        # Combine agent locations and task locations
        currLocsAndGoals = self.locations + self.GoalLocations
        # Total number of locations
        size = len(currLocsAndGoals)
        # Initialize cost matrix
        cmat = np.zeros((size, size))

        for row in range(size):
            for col in range(size):

                # self-loops
                if row == col:
                    continue

                # If the edge is in the include set, its cost is zero
                elif (currLocsAndGoals[row], currLocsAndGoals[col]) in includeE:
                    cmat[row, col] = 0
                    # Special case for next initial positions
                    if row < len(self.locations) and len(self.locations) > col == (row + 1) % len(self.locations):
                        self.includedEdgesRealCost[(currLocsAndGoals[row], currLocsAndGoals[col])] = 0
                    # Calculate Manhattan distance as the real cost
                    else:
                        self.includedEdgesRealCost[(currLocsAndGoals[row], currLocsAndGoals[col])] = self.calculateManhattanDistance(currLocsAndGoals[row], currLocsAndGoals[col])

                # If the edge is in the exclude set, assign a high cost to prohibit its use
                elif (currLocsAndGoals[row], currLocsAndGoals[col]) in excludeE:
                    cmat[row, col] = 99999

                # If it's the next initial position, assign zero cost
                elif row < len(self.locations) and len(self.locations) > col == (row + 1) % len(self.locations):
                    cmat[row, col] = 0

                # If it's not the next initial position, assign a high cost
                elif row < len(self.locations) and len(self.locations) > col != (row + 1) % len(self.locations):
                    cmat[row, col] = 99999

                # If the edge goes from a goal to an initial position, assign zero cost
                elif row >= len(self.locations) > col:
                    cmat[row, col] = 0

                # Default case: calculate Manhattan distance as the cost
                else:
                    cmat[row, col] = self.calculateManhattanDistance(currLocsAndGoals[row], currLocsAndGoals[col])
        return cmat

    def generateMtspPar(self):
        with open("files/Mtsp.par", mode="w+") as fpar:
            fpar.writelines(["PROBLEM_FILE = files/Mtsp.tsp\n"])
            fpar.writelines(["MOVE_TYPE = 5\n"])
            fpar.writelines(["PATCHING_C = 3\n"])
            fpar.writelines(["PATCHING_A = 2\n"])
            fpar.writelines(["RUNS = 10\n"])
            fpar.writelines(["OUTPUT_TOUR_FILE = files/Mtsp.tour\n"])
            fpar.close()

    def generateMtspFile(self, costMatrix):
        nx, ny = costMatrix.shape
        with open("files/Mtsp.tsp", mode="w+") as ftsp:
            ftsp.writelines(["NAME : mtspf\n", "COMMENT : file for mtspf test\n", "TYPE : ATSP\n"])
            ftsp.write("DIMENSION : " + str(nx) + "\n")
            ftsp.writelines(
                ["EDGE_WEIGHT_TYPE : EXPLICIT\n", "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n", "EDGE_WEIGHT_SECTION\n"])
            for ix in range(nx):
                nline = ""
                for iy in range(nx):
                    nline = nline + str(int(costMatrix[(ix, iy)])) + " "
                ftsp.write(nline + "\n")
            ftsp.close()

    def invoke_lkh(self):
        cmd = ["/home/yonikid/Desktop/SimulatorAgents/pRobustCbss/LKH-3.0.11/LKH", "files/Mtsp.par"]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        process.wait()

        # Parse the result
        mtsp_tours = {"Allocations": {}, "tour": []}
        currLocsAndGoals = self.locations + self.GoalLocations

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
                tour.append(currLocsAndGoals[val - 1])
                if first:
                    agent = val - 1
                    currAgentTour.append(currLocsAndGoals[val - 1])
                    first = False

                # If it's a new agent
                elif not first and val <= 5:
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [currLocsAndGoals[val - 1]]
                    agent = val - 1
                else:
                    currAgentTour.append(currLocsAndGoals[val - 1])

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour
            # Create tour as pairs of locations
            mtsp_tours["tour"] = [(tour[i], tour[i + 1]) for i in range(len(tour) - 1)]

            # Adjust cost for included edges
            for loc in mtsp_tours["tour"]:
                mtsp_tours["Cost"] += self.includedEdgesRealCost.get(loc, 0)

        return mtsp_tours

    def calculateManhattanDistance(self, loc1, loc2):
        # loc1 divided by num_of_cols gives row1, remainder gives col1
        row1, col1 = divmod(loc1, self.num_of_cols)
        # loc2 divided by num_of_cols gives row2, remainder gives col2
        row2, col2 = divmod(loc2, self.num_of_cols)
        # Compute Manhattan distance
        return abs(row1 - row2) + abs(col1 - col2)


# print(kBestSequencing([(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)], [122, 124, 126, 128, 130], 1, 12).Solution)