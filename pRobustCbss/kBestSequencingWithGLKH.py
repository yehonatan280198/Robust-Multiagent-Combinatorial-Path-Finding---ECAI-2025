import math
import os
import subprocess
from collections import defaultdict
import numpy as np
from queue import PriorityQueue
from collections import deque

from pRobustCbss.NodeStateConstClasses import State


####################################################### generate Mtsp Parameters file ############################################################
def generateMtspPar():
    lines = ["PROBLEM_FILE = files/Mtsp.gtsp\n", "MOVE_TYPE = 5\n", "PATCHING_C = 3\n",
             "PATCHING_A = 2\n", "RUNS = 10\n", "OUTPUT_TOUR_FILE = files/Mtsp.tour\n"]

    with open("files/Mtsp.par", mode="w+") as fpar:
        fpar.writelines(lines)


####################################################### Create 4 copies of specific goal ############################################################
def createCopyOfGoals(GoalLocations):
    listOfTuplesOfGoals = []
    for goal in GoalLocations:
        for direct in range(4):
            listOfTuplesOfGoals.append((goal, direct))

    return listOfTuplesOfGoals


######################################################### kBestSequencingWithGLKH class ###############################################################
class kBestSequencingWithGLKH:
    def __init__(self, Positions, GoalLocations, dict_of_map_and_dim):
        self.Positions = Positions
        self.OnlyLocOfPosition = [pos for pos, _ in Positions]
        self.AllCopyOfGoals = createCopyOfGoals(GoalLocations)
        self.AllPosAndGoals = self.Positions + self.AllCopyOfGoals

        self.MapAndDims = dict_of_map_and_dim

        self.OPEN = PriorityQueue()  # Priority queue for exploring k-best solutions
        self.included_edges_real_cost = set()  # Stores real costs for included edges

        self.minDist = defaultdict(int)
        self.precomputed_cost = defaultdict(int)

        self.Counter_BFS_For_Test = 0
        self.Counter_Solver_Tsp_For_Test = 0

        self.Precompute_All_The_Costs()

    ######################################################### Find K Best Solution ###############################################################

    def Find_K_Best_Solution(self, k):
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
            # print(optimalSequences)

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
                newExcludeE = excludeE | {
                    (optimalSequences["tour"][index][0][0], optimalSequences["tour"][index][1][0])}
                # Solve again
                PotentialOptimalSequences = self.Solve_Tsp_With_Constraints(newIncludeE, newExcludeE)

                # Validate the new solution by ensuring it respects include/exclude constraints
                if not all(edge in PotentialOptimalSequences["tour"] for edge in newIncludeE):
                    continue

                # Validate the new solution by ensuring it respects include/exclude constraints
                OnlyEdgeOfLocInTour = [(startPos[0], endPos[0]) for startPos, endPos in PotentialOptimalSequences["tour"]]
                if any(edge in OnlyEdgeOfLocInTour for edge in newExcludeE):
                    continue

                # Add the valid solution to the queue
                self.OPEN.put((PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

        return {"Allocations": {}, "tour": [], "Cost": math.inf}

    ######################################################### Solve tsp with constraints ###############################################################
    def Solve_Tsp_With_Constraints(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.Create_Cost_Matrix(includeE, excludeE)
        # Generate LKH parameter file
        generateMtspPar()
        # Generate the MTSP input file
        self.Generate_Mtsp_Problem_File(costMatrix)
        # Run the LKH solver and return the result
        return self.Invoke_GLKH()

    ######################################################### Create cost matrix ###############################################################
    def Create_Cost_Matrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllPosAndGoals), len(self.AllPosAndGoals)))
        for row, (rowLoc, rowDirect) in enumerate(self.AllPosAndGoals):
            for col, (colLoc, colDirect) in enumerate(self.AllPosAndGoals):

                # if not same location
                if rowLoc != colLoc:
                    if ((rowLoc, rowDirect), (colLoc, colDirect)) in includeE:
                        cmat[row, col] = -99999
                    elif (rowLoc, colLoc) in excludeE:
                        cmat[row, col] = 99999
                    else:
                        cmat[row, col] = self.precomputed_cost[((rowLoc, rowDirect), (colLoc, colDirect))]

        return cmat

    ####################################################### Generate mtsp problem file ############################################################
    def Generate_Mtsp_Problem_File(self, costMatrix):
        nx, ny = costMatrix.shape
        numOfGoals = (nx - len(self.Positions)) // 4
        totalSets = len(self.Positions) + numOfGoals
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
            for agent in range(1, len(self.Positions) + 1):
                ftsp.write(f"{agent} {agent} -1\n")

            currSet = len(self.Positions) + 1
            index = len(self.Positions) + 1
            for goal in range(numOfGoals):
                ftsp.write(f"{currSet} {index} {index + 1} {index + 2} {index + 3} -1\n")
                currSet += 1
                index += 4
            ftsp.close()

    ############################################################# Invoke GLKH ####################################################################
    def Invoke_GLKH(self):
        cmd = ["GLKH-1.1/GLKH", "files/Mtsp.par"]
        self.Counter_Solver_Tsp_For_Test += 1
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

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
                goalLoc, goalDirect = self.AllPosAndGoals[val - 1]
                tour.append((goalLoc, goalDirect))
                if first:
                    agent = val - 1
                    currAgentTour.append(goalLoc)
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.Positions):
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [goalLoc]
                    agent = val - 1
                else:
                    currAgentTour.append(goalLoc)

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour
            # Create tour as pairs of locations
            mtsp_tours["tour"] = [(tour[i], tour[i + 1]) for i in range(len(tour) - 1)]

            for index, CurrEdge in enumerate(mtsp_tours["tour"]):
                if CurrEdge in self.included_edges_real_cost:
                    if mtsp_tours["tour"][index + 1][1][0] in self.OnlyLocOfPosition:
                        mtsp_tours["Cost"] += (self.minDist[(
                            mtsp_tours["tour"][index][0], mtsp_tours["tour"][index][1][0])] + 99999)
                    else:
                        mtsp_tours["Cost"] += (self.precomputed_cost[CurrEdge] + 99999)

        return mtsp_tours

    ############################################################# Precompute all the costs ####################################################################
    def Precompute_All_The_Costs(self):
        for i, pos1 in enumerate(self.Positions):
            for j, pos2 in enumerate(self.Positions):
                if i != j and j != (i + 1) % len(self.Positions):
                    self.precomputed_cost[(pos1, pos2)] = 99999
                    self.minDist[(pos1, pos2)] = 99999

        for index, pos in enumerate(self.AllPosAndGoals):
            self.BFS(pos)

    def BFS(self, pos):
        counter_of_reach_goals = 0
        self.Counter_BFS_For_Test += 1
        visited = set()
        S = State(pos)
        queue = deque([S])

        while queue:
            current_state = queue.popleft()

            if current_state.CurPosition in visited:
                continue

            visited.add(current_state.CurPosition)

            if current_state.CurPosition in self.AllCopyOfGoals and current_state.CurPosition != pos:
                self.precomputed_cost[(pos, current_state.CurPosition)] = current_state.g
                self.minDist[(pos, current_state.CurPosition[0])] = (
                    current_state.g if self.minDist[(pos, current_state.CurPosition[0])] == 0
                    else min(self.minDist[(pos, current_state.CurPosition[0])], current_state.g)
                )
                counter_of_reach_goals += 1
                if counter_of_reach_goals == len(self.AllCopyOfGoals):
                    return

            neighbors = self.GetNeighbors(current_state)
            for Sl in neighbors:
                if Sl.CurPosition not in visited:
                    queue.append(Sl)

    def GetNeighbors(self, S):
        neighbors = set()
        loc, direct = S.CurPosition

        candidates = [loc + 1, loc + self.MapAndDims["Cols"], loc - 1, loc - self.MapAndDims["Cols"]]

        # Try moving in the current direction
        loc_after_move = candidates[direct]
        if self.validateMove(loc_after_move, S):
            neighbors.add(State((loc_after_move, direct), S.g + 1, S))

        # Try turning left
        new_direction = (direct - 1) % 4
        neighbors.add(State((loc, new_direction), S.g + 1, S))

        # Try turning right
        new_direction = (direct + 1) % 4
        neighbors.add(State((loc, new_direction), S.g + 1, S))

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
# p = kBestSequencingWithGLKH([(5, 1), (31, 2), (51, 0)], [29, 53], d)
# p.Find_K_Best_Solution(15)
#
# p = kBestSequencingWithGLKH([(50,0), (89,3)], [17, 56], d)
# p.Find_K_Best_Solution(12)
#
# p = kBestSequencingWithGLKH([(74,0)], [41, 80, 5], d)
# p.Find_K_Best_Solution(12)

