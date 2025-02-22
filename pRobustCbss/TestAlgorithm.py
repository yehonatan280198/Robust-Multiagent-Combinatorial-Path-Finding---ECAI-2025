import time
import csv
import ast

from multiprocessing import Process, Queue
from pRobustCbss.Run_pRobustCbss import pRobustCbss

with open("output.csv", mode="w", newline="", encoding="utf-8") as file:
    columns = ["Map", "No collision prob", "Delay prob", "Number of agent", "Number of goals", "Number of successes",
               "AVG elapsed time", "AVG elapsed time (successes only)", "Avg calls to TSP", "Avg calls to BFS"]
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()


def create_map(name):
    file_path = f"/home/yonikid/Desktop/SimulatorAgents/Start-Kit-main/example_problems/OurResearch.domain/{name}"
    with open(file_path, "r") as file:
        lines = file.readlines()
    map_start_index = lines.index("map\n") + 1
    map_lines = lines[map_start_index:]

    currMap = []
    rows, cols = 0, 0
    for line in map_lines:
        cols = len(line.strip())
        rows += 1
        for char in line.strip():
            currMap += [0] if char == "." else [1]

    return {"Rows": rows, "Cols": cols, "Map": currMap}


def read_locs_from_file(mapName, num_of_iter, num_of_agent, num_of_goals):
    file_agents_name = f"Agent_Goal_locations_files/{mapName.split('-')[0].upper()}_Map_Agent_Locs_iteration_{num_of_iter}.txt"
    file_goals_name = f"Agent_Goal_locations_files/{mapName.split('-')[0].upper()}_Map_Goal_Locs_iteration_{num_of_iter}.txt"

    with open(file_agents_name, "r") as f:
        Agents_Positions = [ast.literal_eval(line.strip()) for _, line in zip(range(num_of_agent), f)]

    with open(file_goals_name, "r") as f:
        Goals_Locations = [ast.literal_eval(line.strip()) for _, line in zip(range(num_of_goals), f)]

    return Agents_Positions, Goals_Locations


def run_pRobustCbss(queue, Positions, GoalLocations, No_collision_prob, DelaysProbDict, dict_of_map_and_dim,
                    VerifyAlpha):
    p = pRobustCbss(Positions, GoalLocations, No_collision_prob, DelaysProbDict, dict_of_map_and_dim, VerifyAlpha)
    queue.put(p.Solution)


# mapsList = ["empty-32-32.map", "random-32-32-20.map", "maze-32-32-2.map", "room-32-32-4.map"]
mapsList = ["den312d.map"]
num_of_agentsList = [5]
num_of_goalsList = [10]
# num_of_agentsList = [3, 5, 10, 15, 20, 25]
# num_of_goalsList = [6, 10, 20, 30, 40, 50]
# no_collision_probList = [0.6, 0.7, 0.8]
# delays_probList = [0.05, 0.3]
# num_of_agentsList = [5, 15, 25]
# num_of_goalsList = [10, 30, 50]
no_collision_probList = [0.6]
delays_probList = [0.05]
iterations = 10
verifyAlpha = 0.05
max_time = 60

for map_name in mapsList:
    mapAndDim = create_map(map_name)
    for no_collision_prob in no_collision_probList:
        for delaysProb in delays_probList:
            for numAgents in num_of_agentsList:
                delaysProbDict = {i: delaysProb for i in range(numAgents)}
                for numGoals in num_of_goalsList:

                    statisticConfig = {"Success": 0, "ElapsedTimeOfSuccess": 0, "CallToBFS": 0, "CallToTSP": 0,
                                       "TotalTime": 0}

                    for iteration in range(10):
                        AgentsPositions, GoalsLocations = read_locs_from_file(map_name, iteration, numAgents, numGoals)

                        print(f"map: {map_name}, no collision prob: {no_collision_prob}, delay prob: {delaysProb}")
                        print("AgentsPositions:", AgentsPositions)
                        print("GoalsLocations:", GoalsLocations)

                        queue = Queue()
                        process = Process(target=run_pRobustCbss, args=(
                            queue, AgentsPositions, GoalsLocations, no_collision_prob, delaysProbDict, mapAndDim,
                            verifyAlpha))
                        process.start()
                        start_time = time.time()
                        process.join(timeout=max_time)

                        if process.is_alive():
                            process.terminate()
                            process.join()
                            print(f"Skipped due to timeout after {max_time} seconds.")
                            print("--------------------------------------------------------------------------")
                            statisticConfig["TotalTime"] += max_time
                            continue

                        elapsed_time = round(time.time() - start_time, 2)

                        # If the process completed in time, retrieve the solution
                        solution = queue.get()
                        print("Solution:", solution[0])
                        print("--------------------------------------------------------------------------")

                        statisticConfig["Success"] += 1
                        statisticConfig["ElapsedTimeOfSuccess"] += elapsed_time
                        statisticConfig["CallToTSP"] += solution[1]
                        statisticConfig["CallToBFS"] += solution[2]
                        statisticConfig["TotalTime"] += elapsed_time

                    num_of_success = statisticConfig["Success"] if statisticConfig["Success"] > 0 else 1
                    record = [map_name, no_collision_prob, delaysProb, numAgents, numGoals, statisticConfig["Success"],
                              round(statisticConfig["TotalTime"] / 10, 3),
                              round(statisticConfig["ElapsedTimeOfSuccess"] / num_of_success, 3),
                              round(statisticConfig["CallToTSP"] / num_of_success, 3),
                              round(statisticConfig["CallToBFS"] / num_of_success, 3),
                              ]

                    with open("output.csv", mode="a", newline="", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        writer.writerow(record)
