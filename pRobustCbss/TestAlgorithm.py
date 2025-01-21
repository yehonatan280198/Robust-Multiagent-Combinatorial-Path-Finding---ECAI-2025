import random
import time
import csv

from collections import defaultdict
from multiprocessing import Process, Queue
from pRobustCbss.Run_pRobustCbss import pRobustCbss

columns = ["Map", "No collision prob", "Delay prob", "Number of agent", "Number of goals", "Number of successes",
           "Avg time", "Avg calls to TSP", "Avg calls to BFS"]
with open("output.csv", mode="w", newline="", encoding="utf-8") as file:
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()

config_log_file = "config_log.txt"
with open(config_log_file, mode="w", encoding="utf-8") as log_file:
    log_file.write("Configuration Log\n")
    log_file.write("=" * 50 + "\n\n")


def log_configuration_to_text(map_name, no_collision_prob, delaysProb, numAgents, numGoals, AgentsPositions,
                              GoalsLocations, iteration):
    """Log the current configuration to the text file."""
    with open(config_log_file, mode="a", encoding="utf-8") as log_file:
        log_file.write(f"Iteration: {iteration}\n")
        log_file.write(f"Map: {map_name}\n")
        log_file.write(f"No Collision Probability: {no_collision_prob}\n")
        log_file.write(f"Delays Probability: {delaysProb}\n")
        log_file.write(f"Number of Agents: {numAgents}\n")
        log_file.write(f"Number of Goals: {numGoals}\n")
        log_file.write(f"Agents Positions: {AgentsPositions}\n")
        log_file.write(f"Goals Locations: {GoalsLocations}\n")
        log_file.write("-" * 50 + "\n")

def create_map(name):
    file_path = f"/home/yonikid/Desktop/SimulatorAgents/Start-Kit-main/example_problems/OurResearch.domain/{name}"
    with open(file_path, "r") as file:
        lines = file.readlines()
    map_start_index = lines.index("map\n") + 1
    map_lines = lines[map_start_index:]
    currMap = [0 if char == "." else 1 for line in map_lines for char in line.strip()]
    return {"Rows": 32, "Cols": 32, "Map": currMap}


def create_positions_for_agents_And_Locs_For_Goals(NumAgents, NumGoals, MapAndDim):
    zero_indices = [i for i, value in enumerate(MapAndDim["Map"]) if value == 0]
    chosen_indices_for_agents = random.sample(zero_indices, NumAgents)
    position_for_agents = [(num, random.randint(0, 3)) for num in chosen_indices_for_agents]

    chosen_set = set(chosen_indices_for_agents)
    remaining_indices_for_goals = [idx for idx in zero_indices if idx not in chosen_set]
    location_for_agents = random.sample(remaining_indices_for_goals, NumGoals)

    return position_for_agents, location_for_agents


def run_pRobustCbss(queue, Positions, GoalLocations, No_collision_prob, DelaysProbDict, dict_of_map_and_dim,
                    VerifyAlpha):
    p = pRobustCbss(Positions, GoalLocations, No_collision_prob, DelaysProbDict, dict_of_map_and_dim, VerifyAlpha)
    queue.put(p.Solution)


# mapAndDim = create_map()
mapsList = ["empty-32-32.map", "random-32-32-20.map", "maze-32-32-2.map", "room-32-32-4.map"]  # 4
num_of_agentsList = [3, 5, 10, 15, 20, 25]  # 6
num_of_goalsList = [6, 10, 20, 30, 40, 50]  # 6
no_collision_probList = [0.6, 0.7, 0.8]  # 3
delays_probList = [0.05, 0.3]  # 2
iterations = 10  # 10
verifyAlpha = 0.05
max_time = 60

dict_of_successes = defaultdict(lambda: [0, 0, 0, 0])

for map_name in mapsList:
    mapAndDim = create_map(map_name)
    for no_collision_prob in no_collision_probList:
        for delaysProb in delays_probList:
            for numAgents in num_of_agentsList:
                delaysProbDict = {i: delaysProb for i in range(numAgents)}
                for numGoals in num_of_goalsList:

                    for iteration in range(iterations):
                        AgentsPositions, GoalsLocations = create_positions_for_agents_And_Locs_For_Goals(numAgents,
                                                                                                         numGoals,
                                                                                                         mapAndDim)

                        log_configuration_to_text(
                            map_name, no_collision_prob, delaysProb, numAgents, numGoals, AgentsPositions, GoalsLocations, iteration
                        )

                        print("AgentsPositions:", AgentsPositions)
                        print("GoalsLocations:", GoalsLocations)
                        successful_runs = True

                        queue = Queue()
                        start_time = time.time()
                        process = Process(target=run_pRobustCbss, args=(
                        queue, AgentsPositions, GoalsLocations, no_collision_prob, delaysProbDict, mapAndDim,
                        verifyAlpha))
                        process.start()
                        process.join(timeout=max_time)

                        if process.is_alive():
                            process.terminate()
                            process.join()
                            print(f"Skipped due to timeout after {max_time} seconds.")
                            print("--------------------------------------------------------------------------")
                            successful_runs = False
                            continue

                        end_time = time.time()
                        elapsed_time = round(end_time - start_time, 2)

                        # If the process completed in time, retrieve the solution
                        solution = queue.get()
                        print("Solution:", solution)
                        print("--------------------------------------------------------------------------")

                        if successful_runs:
                            dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][0] += 1
                            dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][
                                1] += elapsed_time
                            dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][2] += \
                            solution[1]  # TSP
                            dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][3] += \
                            solution[2]  # BFS

                    count_success = dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][0]
                    if count_success > 0:
                        record = [
                            map_name,
                            no_collision_prob,
                            delaysProb,
                            numAgents,
                            numGoals,
                            count_success,
                            round(dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][
                                      1] / count_success, 3),
                            round(dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][
                                      2] / count_success, 3),
                            round(dict_of_successes[(map_name, no_collision_prob, delaysProb, numAgents, numGoals)][
                                      3] / count_success, 3),
                        ]
                        with open("output.csv", mode="a", newline="", encoding="utf-8") as file:
                            writer = csv.writer(file)
                            writer.writerow(record)

                    else:
                        break
