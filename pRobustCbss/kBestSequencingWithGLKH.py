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
    lines = [f"PROBLEM_FILE = {os.getcwd()}/files/Mtsp.gtsp\n", "MOVE_TYPE = 5\n", "PATCHING_C = 3\n",
             "PATCHING_A = 2\n", "RUNS = 10\n", f"OUTPUT_TOUR_FILE = {os.getcwd()}/files/Mtsp.tour\n"]

    with open(f"{os.getcwd()}/files/Mtsp.par", mode="w+") as filePar:
        filePar.writelines(lines)


####################################################### Create 4 copies of specific goal ############################################################
def createCopyOfGoals(GoalLocations):
    return [(goal, direct) for goal in GoalLocations for direct in range(4)]


######################################################### kBestSequencingWithGLKH class ###############################################################
class kBestSequencingWithGLKH:
    def __init__(self, Positions, GoalLocations, dict_of_map_and_dim):
        self.Positions = Positions
        self.AllCopyOfGoals = createCopyOfGoals(GoalLocations)
        self.AllPosAndGoals = self.Positions + self.AllCopyOfGoals

        self.MapAndDims = dict_of_map_and_dim
        self.OPEN = PriorityQueue()
        self.Solutions = {}

        self.Counter_BFS_For_Test = 0
        self.Counter_Solver_Tsp_For_Test = 0

        self.precomputed_cost = self.Precompute_All_The_Costs()

    ######################################################### Find K Best Solution ###############################################################

    def Find_K_Best_Solution(self, k):
        # Save the current working directory to return to it later
        current_dir = os.getcwd()
        os.chdir('/home/yonikid/Desktop/SimulatorAgents/pRobustCbss')

        # If k is 1, find and return the best (first) solution
        if k == 1:
            # Save the first allocation along with the include/exclude sets
            self.Solutions[k] = (set(), set(), self.Solve_Tsp_With_Constraints(set(), set()))
            # Return optimal allocation
            return self.Solutions[k][2]

        # Retrieve the previous solution for k-1
        includeE, excludeE, optimalSequences = self.Solutions[k - 1]

        # Iterate over the edges in the current solution's allocation
        for index, (v0, v1) in enumerate(optimalSequences["Alloc_edges"]):
            # Create a new include set by adding edges up to the current index
            newIncludeE = includeE | set(optimalSequences["Alloc_edges"][:index])
            # Create a new exclude set by adding the current edge (only locs)
            newExcludeE = excludeE | {(v0, v1)}

            if newIncludeE.intersection(newExcludeE):
                continue

            # Solve the TSP problem with the new constraints
            PotentialOptimalSequences = self.Solve_Tsp_With_Constraints(newIncludeE, newExcludeE)

            # Validate the new solution by ensuring it respects the include constraints
            if not all(edge in PotentialOptimalSequences["Alloc_edges"] for edge in newIncludeE):
                continue

            # Ensure the solution respects the exclude constraints
            if any(edge in PotentialOptimalSequences["Alloc_edges"] for edge in newExcludeE):
                continue

            # Add the valid solution to the priority queue
            self.OPEN.put((PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

        # If the priority queue is empty, return a default solution indicating no more allocations
        if self.OPEN.empty():
            return {"Allocations": {}, "Alloc_edges": [], "Cost": math.inf}

        # Retrieve the next best solution from the queue
        _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
        # Save the new solution in the list of solutions
        self.Solutions[k] = (includeE, excludeE, optimalSequences)
        # Return to the original directory
        os.chdir(current_dir)
        # Return the optimal solution found
        return optimalSequences

    ######################################################### Solve tsp with constraints ###############################################################
    def Solve_Tsp_With_Constraints(self, includeE, excludeE):
        # Create the cost matrix
        costMatrix = self.Create_Cost_Matrix(includeE, excludeE)
        # Generate LKH parameter file
        generateMtspPar()
        # Generate the MTSP input file
        self.Generate_Mtsp_Problem_File(costMatrix)
        # Run the LKH solver and return the result
        return self.Invoke_GLKH(includeE)

    ######################################################### Create cost matrix ###############################################################
    def Create_Cost_Matrix(self, includeE, excludeE):
        cmat = np.zeros((len(self.AllPosAndGoals), len(self.AllPosAndGoals)))
        for row, (rowLoc, rowDirect) in enumerate(self.AllPosAndGoals):
            for col, (colLoc, colDirect) in enumerate(self.AllPosAndGoals):

                if rowLoc != colLoc:
                    if (rowLoc, colLoc) in includeE:
                        cmat[row, col] = -(100000 - self.precomputed_cost[((rowLoc, rowDirect), (colLoc, colDirect))])
                    elif (rowLoc, colLoc) in excludeE:
                        cmat[row, col] = 99999
                    elif (row < len(self.Positions) and col < len(self.Positions)) or (row >= len(self.Positions) > col):
                        cmat[row, col] = 0
                    else:
                        cmat[row, col] = self.precomputed_cost[((rowLoc, rowDirect), (colLoc, colDirect))]

        return cmat

    ####################################################### Generate mtsp problem file ############################################################
    def Generate_Mtsp_Problem_File(self, costMatrix):
        nx, ny = costMatrix.shape
        numOfGoals = (nx - len(self.Positions)) // 4
        totalSets = len(self.Positions) + numOfGoals
        with open(f"{os.getcwd()}/files/Mtsp.gtsp", mode="w+") as FileGtsp:
            FileGtsp.writelines(["NAME : mtspf\n", "TYPE : AGTSP\n"])
            FileGtsp.write("DIMENSION : " + str(nx) + "\n")
            FileGtsp.write("GTSP_SETS : " + str(totalSets) + "\n")
            FileGtsp.writelines(
                ["EDGE_WEIGHT_TYPE : EXPLICIT\n", "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n", "EDGE_WEIGHT_SECTION\n"])
            for ix in range(nx):
                nline = ""
                for iy in range(nx):
                    nline = nline + str(int(costMatrix[(ix, iy)])) + " "
                FileGtsp.write(nline + "\n")

            FileGtsp.write("GTSP_SET_SECTION\n")
            for agent in range(1, len(self.Positions) + 1):
                FileGtsp.write(f"{agent} {agent} -1\n")

            currSet = len(self.Positions) + 1
            index = len(self.Positions) + 1
            for goal in range(numOfGoals):
                FileGtsp.write(f"{currSet} {index} {index + 1} {index + 2} {index + 3} -1\n")
                currSet += 1
                index += 4
            FileGtsp.close()

    ############################################################# Invoke GLKH ####################################################################
    def Invoke_GLKH(self, includeE):
        print("hi")
        cmd = [f"{os.getcwd()}/GLKH-1.1/GLKH", f"{os.getcwd()}/files/Mtsp.par"]
        self.Counter_Solver_Tsp_For_Test += 1
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        process.wait()

        mtsp_tours = {"Allocations": {}, "Alloc_edges": []}

        with open(f"{os.getcwd()}/files/Mtsp.tour", mode="r") as FileTour:
            lines = FileTour.readlines()
            mtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6  # Starting index of the tour in the output file
            val = int(lines[ix])
            currAgentTour = []
            agent = -1
            first = True

            a = []
            # Read until the end of the tour
            while val != -1:
                a.append(self.AllPosAndGoals[val - 1])
                goalLoc, goalDir = self.AllPosAndGoals[val - 1]
                if first:
                    agent = val - 1
                    currAgentTour.append((goalLoc, goalDir))
                    first = False

                # If it's a new agent
                elif not first and val <= len(self.Positions):
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [(goalLoc, goalDir)]
                    agent = val - 1
                else:
                    mtsp_tours["Alloc_edges"].append((currAgentTour[-1][0], goalLoc))
                    currAgentTour.append((goalLoc, goalDir))

                ix = ix + 1
                val = int(lines[ix])

            # Add the final agent's tour
            mtsp_tours["Allocations"][agent] = currAgentTour

            mtsp_tours["Cost"] += (len(includeE)*100000)

        return mtsp_tours

    ############################################################# Precompute all the costs ####################################################################
    def Precompute_All_The_Costs(self):
        precomputed_cost = defaultdict(lambda: 100000)

        for index, pos in enumerate(self.AllPosAndGoals):
            print(index)
            self.BFS(pos, precomputed_cost)

        return precomputed_cost

    def BFS(self, pos, precomputed_cost):
        self.Counter_BFS_For_Test += 1
        counter_of_reach_goals = 0
        visited = set()
        S = State(pos)
        queue = deque([S])

        while queue:
            current_state = queue.popleft()
            if current_state.CurPosition in visited:
                continue

            visited.add(current_state.CurPosition)

            if current_state.CurPosition in self.AllCopyOfGoals and current_state.CurPosition != pos:
                precomputed_cost[(pos, current_state.CurPosition)] = current_state.g
                counter_of_reach_goals += 1
                if counter_of_reach_goals == len(self.AllCopyOfGoals) or (counter_of_reach_goals == len(self.AllCopyOfGoals) - 1 and pos in self.AllCopyOfGoals):
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

# d = {"Rows": 32, "Cols": 32, "Map": [0 for _ in range(32 * 32)]}
# p = kBestSequencingWithGLKH([(955, 1), (347, 0), (349, 2), (395, 3), (7, 1)], [887, 43, 742, 992, 540, 338], d)
# for i in range(1, 3):
#     print(i, p.Find_K_Best_Solution(i))

# p = kBestSequencingWithGLKH([(848, 1), (941, 0), (39, 2), (152, 2), (953, 3)], [110, 308, 738, 460, 1020, 956, 334, 264, 459, 476, 727, 49, 810, 743, 685, 395, 377, 555, 320, 642], d)
# # # p = kBestSequencingWithGLKH([(850, 0), (564, 0), (557, 1), (337, 1), (252, 2)], [248, 885, 318, 64, 75, 614, 893, 147, 770, 62, 204, 353, 926, 34, 796, 111, 28, 810, 776, 1008], d)
# p.Find_K_Best_Solution(4)

# d = {"Rows": 12, "Cols": 12, "Map": [0 for _ in range(12 * 12)]}
# p = kBestSequencingWithGLKH([(5, 1), (55, 2), (75, 0)], [29, 53, 77], d)
# for i in range(1, 62):
#     print(i, p.Find_K_Best_Solution(i))
# #
# # p = kBestSequencingWithGLKH([(31, 2)], [5, 29, 53], d)
# p = kBestSequencingWithGLKH([(5, 1), (31, 2), (51, 0)], [29, 53], d)
# for i in range(1, 14):
#     print(p.Find_K_Best_Solution(i))
#
# p = kBestSequencingWithGLKH([(50,0), (89,3)], [17, 56], d)
# for i in range(1, 8):
#     print(p.Find_K_Best_Solution(i))
#
# p = kBestSequencingWithGLKH([(74,0)], [41, 80, 5], d)
# for i in range(1, 8):
#     print(p.Find_K_Best_Solution(i))
