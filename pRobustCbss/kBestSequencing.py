import subprocess
import numpy as np


def kBestSequencing(locations, taskLocs, k):
    costMatrix = createCostMatrix(locations, taskLocs)
    generateMtspPar()
    generateMtspFile(costMatrix)
    print(invoke_lkh())


def createCostMatrix(locations, taskLocs):
    coordinates = locations + taskLocs
    size = len(coordinates)
    cmat = np.zeros((size, size))
    for row in range(size):
        for col in range(size):
            if row == col:
                continue
            elif row < len(locations) and col < len(locations):
                cmat[row, col] = 0
            elif row >= len(locations) > col:
                cmat[row, col] = 0
            else:
                cmat[row, col] = abs(coordinates[row] // 12 - coordinates[col] // 12) + abs(
                    coordinates[row] % 12 - coordinates[col] % 12)
    return cmat


def generateMtspPar():
    with open("Mtsp.par", mode="w+") as fpar:
        fpar.writelines(["PROBLEM_FILE = Mtsp.tsp\n"])
        fpar.writelines(["MOVE_TYPE = 5\n"])
        fpar.writelines(["PATCHING_C = 3\n"])
        fpar.writelines(["PATCHING_A = 2\n"])
        fpar.writelines(["RUNS = 10\n"])
        fpar.writelines(["OUTPUT_TOUR_FILE = Mtsp.tour\n"])
        fpar.close()


def generateMtspFile(costMatrix):
    nx, ny = costMatrix.shape
    with open("Mtsp.tsp", mode="w+") as ftsp:
        ftsp.writelines(["NAME : mtspf\n", "COMMENT : file for mtspf test\n", "TYPE : ATSP\n"])
        ftsp.write("DIMENSION : " + str(nx) + "\n")
        ftsp.writelines(["EDGE_WEIGHT_TYPE : EXPLICIT\n", "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n", "EDGE_WEIGHT_SECTION\n"])
        for ix in range(nx):
            nline = ""
            for iy in range(nx):
                nline = nline + str(int(costMatrix[(ix, iy)])) + " "
            ftsp.write(nline + "\n")
        ftsp.close()


def invoke_lkh():
    lb_cost = np.inf  # meaningless number
    cmd = ["/home/yonikid/Desktop/SimulatorAgents/pRobustCbss/LKH-3.0.11/LKH", "Mtsp.par"]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    for line in process.stdout:
        line_str = line.decode('UTF-8')
        if line_str[0:11] == "Lower bound":
            temp1 = line_str.split(",")
            temp2 = temp1[0].split("=")
            lb_cost = float(temp2[1])  # lower bound from LKH.
    process.wait()  # otherwise, subprocess run concurrently...

    ### get result
    mtsp_tours = {}
    with open("Mtsp.tour", mode="r") as fres:
        lines = fres.readlines()
        mtsp_tours["Cost"] = int(lines[1].split("=")[1])
        ix = 6
        val = int(lines[ix])
        curr_tour = []
        tour = []
        first = True
        while val != -1:
            tour.append(val-1)
            if first:
                curr_tour.append(val - 1)
                first = False
            if not first and val <= 5:
                mtsp_tours[str(curr_tour[0])] = curr_tour
                curr_tour = [val - 1]
            else:
                curr_tour.append(val - 1)

            ix = ix + 1
            val = int(lines[ix])

        mtsp_tours[str(tour[0])] = curr_tour
        mtsp_tours["tour"] = tour

    return mtsp_tours


kBestSequencing([2, 4, 6, 8, 10], [122, 124, 126, 128, 130], 3)
