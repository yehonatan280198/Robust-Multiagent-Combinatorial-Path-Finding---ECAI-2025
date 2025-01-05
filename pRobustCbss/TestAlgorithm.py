import random
import time

from collections import defaultdict
from multiprocessing import Process, Queue
from pRobustCbss.Run_pRobustCbss import pRobustCbss


def create_map():
    file_path = "/home/yonikid/Desktop/SimulatorAgents/Start-Kit-main/example_problems/OurResearch.domain/empty-32-32.map"
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


def run_pRobustCbss(queue, Positions, GoalLocations, no_collision_prob, delaysProb, dict_of_map_and_dim, verifyAlpha):

    p = pRobustCbss(Positions, GoalLocations, no_collision_prob, delaysProb, dict_of_map_and_dim, verifyAlpha)
    queue.put(dict(p.Solution))


# cases = [[True, False, False], [False, False, False], [False, True, False], [False, False, True], [False, True, True]]
# cases = [[False, True, False], [False, True, True]]


mapAndDim = create_map()
num_of_agentsList = [3, 5, 10, 15, 20, 25]
num_of_goalsList = [6, 10, 20, 30, 40, 50]
no_collision_prob = 0.6
verifyAlpha = 0.05
samples = 10
max_time = 120

dict_of_successes = defaultdict(lambda: [0, 0])


for numAgents in num_of_agentsList:
    for numGoals in num_of_goalsList:
        delaysProb = {i: 0.1 for i in range(numAgents)}

        for sample in range(samples):
            AgentsPositions, GoalsLocations = create_positions_for_agents_And_Locs_For_Goals(numAgents, numGoals, mapAndDim)
            print("AgentsPositions:", AgentsPositions)
            print("GoalsLocations:", GoalsLocations)

            successful_runs = True

            queue = Queue()
            start_time = time.time()
            process = Process(target=run_pRobustCbss, args=(queue, AgentsPositions, GoalsLocations, no_collision_prob, delaysProb, mapAndDim, verifyAlpha))
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
            elapsed_time = round(end_time - start_time,2)


            # If the process completed in time, retrieve the solution
            solution = queue.get()
            print("Solution:", solution)
            print("--------------------------------------------------------------------------")

            if successful_runs:
                dict_of_successes[(numAgents, numGoals)][0] += 1
                dict_of_successes[(numAgents, numGoals)][1] += elapsed_time

        if dict_of_successes[(numAgents, numGoals)][0] > 0:
            dict_of_successes[(numAgents, numGoals)][1] = dict_of_successes[(numAgents, numGoals)][1] / dict_of_successes[(numAgents, numGoals)][0]

        print(dict_of_successes)
