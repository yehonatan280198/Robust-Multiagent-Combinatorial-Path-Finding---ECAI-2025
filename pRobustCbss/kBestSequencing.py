import math
import os
import subprocess
from collections import deque, defaultdict

import numpy as np
from queue import PriorityQueue
from pRobustCbss.NodeStateConstClasses import State


def generateMtspPar():
    lines = ["PROBLEM_FILE = files/Mtsp.tsp\n", "MOVE_TYPE = 5\n", "PATCHING_C = 3\n",
             "PATCHING_A = 2\n", "RUNS = 10\n", "OUTPUT_TOUR_FILE = files/Mtsp.tour\n"]

    with open("files/Mtsp.par", mode="w+") as fpar:
        fpar.writelines(lines)


def generateMtspFile(costMatrix):
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


class kBestSequencing:
    def __init__(self, Positions, GoalLocations, dict_of_map_and_dim):
        self.Positions = Positions
        self.OnlyLocOfPosition = [pos for pos, _ in Positions]  # Extract only locations, ignoring direction
        self.GoalLocations = GoalLocations  # Locations of goals
        self.AllLocPosAndGoals = self.OnlyLocOfPosition + self.GoalLocations

        self.MapAndDims = dict_of_map_and_dim

        self.OPEN = PriorityQueue()  # Priority queue for exploring k-best solutions
        self.included_edges_real_cost = set()  # Stores real costs for included edges

        self.Counter_BFS_For_Test = 0
        self.Counter_Solver_Tsp_For_Test = 0

        # Precompute Manhattan distances between all points
        self.precomputed_cost = defaultdict(int)

        self.Precompute_All_The_Costs()

    def run(self, k):
        current_dir = os.getcwd()
        os.chdir('/home/yonikid/Desktop/SimulatorAgents/pRobustCbss')

        # Initialize sets for included and excluded edges
        includeE, excludeE = set(), set()
        # Solve the problem for the initial setup
        optimalSequences = self.Solve_Tsp_With_Constraints(includeE, excludeE)
        # Add to queue
        self.OPEN.put((optimalSequences["Cost"], (includeE, excludeE, optimalSequences)))
        # List to store k-best solutions
        S = []

        while not self.OPEN.empty():
            # Pop the best sequence
            _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
            # Append the optimal sequence to results
            S.append(optimalSequences)
            print(optimalSequences)

            # Stop if k solutions are found
            if len(S) == k:
                os.chdir(current_dir)
                return S[k - 1]

            # Generate new potential solutions by varying include/exclude sets
            for index in range(len(optimalSequences["tour"])):
                self.included_edges_real_cost = set()
                # Add edges to include set
                newIncludeE = includeE | set(optimalSequences["tour"][:index])
                for CurrEdge in newIncludeE:
                    self.included_edges_real_cost.add(CurrEdge)
                # Add the current edge to exclude set
                newExcludeE = excludeE | {optimalSequences["tour"][index]}
                # Solve again
                PotentialOptimalSequences = self.Solve_Tsp_With_Constraints(newIncludeE, newExcludeE)

                # Validate the new solution by ensuring it respects include/exclude constraints
                if not all(edge in PotentialOptimalSequences["tour"] for edge in newIncludeE):
                    continue

                # Validate the new solution by ensuring it respects include/exclude constraints
                if any(edge in PotentialOptimalSequences["tour"] for edge in newExcludeE):
                    continue

                # Add the valid solution to the queue
                self.OPEN.put(
                    (PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

        return {"Allocations": {}, "tour": [], "Cost": math.inf}

    def Solve_Tsp_With_Constraints(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.Create_Cost_Matrix(includeE, excludeE)
        # Generate LKH parameter file
        generateMtspPar()
        # Generate the MTSP input file
        generateMtspFile(costMatrix)
        # Run the LKH solver and return the result
        return self.invoke_lkh()

    def Create_Cost_Matrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllLocPosAndGoals), len(self.AllLocPosAndGoals)))
        for row, rowLoc in enumerate(self.AllLocPosAndGoals):
            for col, colLoc in enumerate(self.AllLocPosAndGoals):

                # if not same location
                if rowLoc != colLoc:
                    if (rowLoc, colLoc) in includeE:
                        cmat[row, col] = -99999
                    elif (rowLoc, colLoc) in excludeE:
                        cmat[row, col] = 99999
                    else:
                        cmat[row, col] = self.precomputed_cost[(rowLoc, colLoc)]

        return cmat

    def invoke_lkh(self):
        cmd = ["/home/yonikid/Desktop/SimulatorAgents/pRobustCbss/LKH-3.0.11/LKH", "files/Mtsp.par"]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        process.wait()

        # Parse the result
        mtsp_tours = {"Allocations": {}, "tour": []}

        with open("files/Mtsp.tour", mode="r") as fres:
            lines = fres.readlines()
            mtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6  # Starting index of the tour in the output file
            val = int(lines[ix])
            currAgentTour = []
            tour = []
            agent = -1
            first = True

            # Read until the end of the tour
            while val != -1:
                tour.append(self.AllLocPosAndGoals[val - 1])
                if first:
                    agent = val - 1
                    currAgentTour.append(self.AllLocPosAndGoals[val - 1])
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.OnlyLocOfPosition):
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [self.AllLocPosAndGoals[val - 1]]
                    agent = val - 1
                else:
                    currAgentTour.append(self.AllLocPosAndGoals[val - 1])

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour
            # Create tour as pairs of locations
            mtsp_tours["tour"] = [(tour[i], tour[i + 1]) for i in range(len(tour) - 1)]

            # Adjust cost for included edges
            for index, CurrEdge in enumerate(mtsp_tours["tour"]):
                if CurrEdge in self.included_edges_real_cost:
                    mtsp_tours["Cost"] += (self.precomputed_cost[CurrEdge] + 99999)

        return mtsp_tours

    def Precompute_All_The_Costs(self):
        for i, pos1 in enumerate(self.Positions):
            for j, pos2 in enumerate(self.Positions):
                if i != j and j != (i + 1) % len(self.Positions):
                    self.precomputed_cost[(pos1[0], pos2[0])] = 99999

        for index, pos in enumerate(self.AllLocPosAndGoals):
            self.BFS(pos)

    def BFS(self, pos):
        counter_of_reach_goals = 0
        self.Counter_BFS_For_Test += 1
        visited = set()
        S = State((pos, 0))
        queue = deque([S])

        while queue:
            current_state = queue.popleft()

            if current_state.CurPosition in visited:
                continue

            visited.add(current_state.CurPosition)

            if current_state.CurPosition[0] in self.GoalLocations and current_state.CurPosition[0] != pos:
                self.precomputed_cost[(pos, current_state.CurPosition[0])] = current_state.g
                counter_of_reach_goals += 1
                if counter_of_reach_goals == len(self.GoalLocations):
                    return

            neighbors = self.GetNeighbors(current_state)
            for Sl in neighbors:
                if Sl.CurPosition not in visited:
                    queue.append(Sl)

    def GetNeighbors(self, S):
        neighbors = set()
        loc, _ = S.CurPosition

        candidates = [loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"]]

        for loc_after_move in candidates:
            if self.validateMove(loc_after_move, S):
                neighbors.add(State((loc_after_move, 0), S.g + 1, S))

        return neighbors

    def validateMove(self, loc_after_move, S):
        # Extract the agent's location and direction before taking the next step
        loc, _ = S.CurPosition

        # If the agent is at the top or bottom boundary, it cannot move up or down
        if not (0 <= loc_after_move < self.MapAndDims["Cols"] * self.MapAndDims["Rows"]):
            return False

        # If the agent is at the right boundary, it cannot move right
        if loc % self.MapAndDims["Cols"] == self.MapAndDims["Cols"] - 1 and loc_after_move % \
                self.MapAndDims["Cols"] == 0:
            return False

        # If the agent is at the left boundary, it cannot move left
        if loc % self.MapAndDims["Cols"] == 0 and loc_after_move % self.MapAndDims["Cols"] == \
                self.MapAndDims["Cols"] - 1:
            return False

        if self.MapAndDims["Map"][loc_after_move] != 0:
            return False

        return True


# d = {"Rows": 12, "Cols": 12, "Map": [0 for _ in range(12 * 12)]}
# p = kBestSequencing([(5, 1), (31, 2), (51, 0)], [29, 53], d)
# p.run(15)
#
# p = kBestSequencing([(50,0), (89,3)], [17, 56], d)
# p.run(12)
#
# p = kBestSequencing([(74,0)], [41, 80, 5], d)
# p.run(12)
