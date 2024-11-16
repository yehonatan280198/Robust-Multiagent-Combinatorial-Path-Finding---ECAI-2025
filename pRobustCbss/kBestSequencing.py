import subprocess

import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from queue import PriorityQueue


class kBestSequencing:
    def __init__(self, locations, taskLocs, k, num_of_cols):
        self.locations = [loc for loc, direct in locations]
        self.taskLocs = taskLocs
        self.k = k
        self.num_of_cols = num_of_cols
        self.OPEN = PriorityQueue()

        self.S = self.run()

    def run(self):
        includeE, excludeE = set(), set()
        optimalSequences = self.solveRtsp(includeE, excludeE)
        self.OPEN.put((optimalSequences["Cost"], (includeE, excludeE, optimalSequences)))
        S = list()

        while not self.OPEN.empty():
            _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
            S.append(optimalSequences)
            if len(S) == self.k:
                return S[self.k - 1]

            indexEdges = optimalSequences["tour"]
            for index, edge in enumerate(indexEdges):
                newIncludeE = includeE | set(indexEdges[:index])
                newExcludeE = excludeE | {indexEdges[index]}
                PotentialOptimalSequences = self.solveRtsp(newIncludeE, newExcludeE)
                if (all(edge not in PotentialOptimalSequences["tour"] for edge in newExcludeE) and
                        all(edge in PotentialOptimalSequences["tour"] for edge in newIncludeE)):
                    self.OPEN.put(
                        (PotentialOptimalSequences["Cost"], (newIncludeE, newExcludeE, PotentialOptimalSequences)))

    def solveRtsp(self, includeE, excludeE):
        costMatrix = self.createCostMatrix(includeE, excludeE)
        self.generateMtspPar()
        self.generateMtspFile(costMatrix)
        return self.invoke_lkh()

    def createCostMatrix(self, includeE, excludeE):
        currLocsAndGoals = self.locations + self.taskLocs
        size = len(currLocsAndGoals)
        cmat = np.zeros((size, size))
        for row in range(size):
            for col in range(size):
                if row == col:
                    continue
                elif (currLocsAndGoals[row], currLocsAndGoals[col]) in excludeE:
                    cmat[row, col] = 99999
                elif row < len(self.locations) and len(self.locations) > col == (row + 1) % len(self.locations):
                    cmat[row, col] = 0
                elif row < len(self.locations) and len(self.locations) > col != (row + 1) % len(self.locations):
                    cmat[row, col] = 99999
                elif row >= len(self.locations) > col:
                    cmat[row, col] = 0
                else:
                    cmat[row, col] = abs(
                        currLocsAndGoals[row] // self.num_of_cols - currLocsAndGoals[col] // self.num_of_cols) + abs(
                        currLocsAndGoals[row] % self.num_of_cols - currLocsAndGoals[col] % self.num_of_cols)
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

        ### get result
        mtsp_tours = {"Allocations": {}, "tour": []}
        currLocsAndGoals = self.locations + self.taskLocs

        with open("files/Mtsp.tour", mode="r") as fres:
            lines = fres.readlines()
            mtsp_tours["Cost"] = int(lines[1].split("=")[1])
            ix = 6
            val = int(lines[ix])
            currAgentTour = []
            tour = []
            agent = -1
            first = True
            while val != -1:
                tour.append(currLocsAndGoals[val - 1])
                if first:
                    agent = val - 1
                    currAgentTour.append(currLocsAndGoals[val - 1])
                    first = False
                elif not first and val <= 5:
                    mtsp_tours["Allocations"][agent] = currAgentTour
                    currAgentTour = [currLocsAndGoals[val - 1]]
                    agent = val - 1
                else:
                    currAgentTour.append(currLocsAndGoals[val - 1])

                ix = ix + 1
                val = int(lines[ix])

            mtsp_tours["Allocations"][agent] = currAgentTour
            mtsp_tours["tour"] = [(tour[i], tour[i + 1]) for i in range(len(tour) - 1)]

        return mtsp_tours


# print(kBestSequencing([(2, 0), (4, 0), (6, 0), (8, 0), (10, 0)], [122, 124, 126, 128, 130], 1, 12).S)

#     def run(self):
#         includeE, excludeE = set(), set()
#         optimalSequences = self.solveRtsp(includeE, excludeE)
#         self.OPEN.put((optimalSequences["Cost"], (includeE, excludeE, optimalSequences)))
#         S = list()
#
#         while not self.OPEN.empty():
#             _, (includeE, excludeE, optimalSequences) = self.OPEN.get()
#             S.append(optimalSequences)
#             if len(S) == self.k:
#                 return S[self.k-1]
#
#     def solveRtsp(self, includeE, excludeE):
#         coordinates = self.locations + self.taskLocs
#         data = self.create_data_model(includeE, excludeE)
#         manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']), data['num_vehicles'], data['depot'], data['depot'])
#         routing = pywrapcp.RoutingModel(manager)
#
#         def distance_callback(from_index, to_index):
#             # Returns the distance between the two nodes
#             from_node = manager.IndexToNode(from_index)
#             to_node = manager.IndexToNode(to_index)
#             return data['distance_matrix'][from_node][to_node]
#
#         transit_callback_index = routing.RegisterTransitCallback(distance_callback)
#         routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
#
#         search_parameters = pywrapcp.DefaultRoutingSearchParameters()
#         # search_parameters.solution_limit = 100  # Limit to 10 solutions
#         search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
#         search_parameters.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
#         search_parameters.time_limit.seconds = 1  # Optional time limit
#
#         solution = routing.SolveWithParameters(search_parameters)
#
#         if solution:
#             dictSeq = {}
#             sumOfCost = 0
#             sequenceDict = {}
#             for vehicle_id in range(data['num_vehicles']):
#                 index = routing.Start(vehicle_id)
#                 route_distance = 0
#                 route = []
#                 while not routing.IsEnd(index):
#                     node_index = manager.IndexToNode(index)
#                     route.append(coordinates[node_index])
#                     previous_index = index
#                     index = solution.Value(routing.NextVar(index))
#                     route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
#                 route.append(coordinates[manager.IndexToNode(index)])  # End node
#                 sumOfCost += route_distance
#                 sequenceDict[vehicle_id] = route[:-1]
#
#             dictSeq["Cost"] = sumOfCost
#             dictSeq["Sequences"] = sequenceDict
#             return dictSeq
#         else:
#             print("No solution found.")
#
#     def create_data_model(self, includeE, excludeE):
#         coordinates = self.locations + self.taskLocs
#         size = len(coordinates)
#         cmat = [[0 for _ in range(size)] for _ in range(size)]
#         for row in range(size):
#             for col in range(size):
#                 if row == col:
#                     continue
#                 elif (row, col) in excludeE:
#                     cmat[row][col] = 99999
#                 elif row < len(self.locations) and col < len(self.locations):
#                     cmat[row][col] = 0
#                 elif row >= len(self.locations) > col:
#                     cmat[row][col] = 0
#                 else:
#                     cmat[row][col] = abs(coordinates[row] // self.num_of_cols - coordinates[col] // self.num_of_cols) + abs(
#                         coordinates[row] % self.num_of_cols - coordinates[col] % self.num_of_cols)
#
#         data = {'distance_matrix': cmat, 'num_vehicles': len(self.locations), 'depot': list(range(len(self.locations)))}
#         return data
#
#
# print(kBestSequencing([(2,0), (4,0), (6,0), (8,0), (10,0)], [122, 124, 126, 128, 130], 1, 12).S)
