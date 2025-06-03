import os
import random
import time
import csv
import ast
import sys

from multiprocessing import Process, Queue
from Run_Tuning_Delays import RobustCbssTuningDelays
from Run_Simulation import Run_Simulation


####################################################### Create Map #################################################################################
def create_map(map_name):
    file_path = f"OurResearch.domain/{map_name}.map"
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


####################################################### Global Variables ######################################################################
mapName = sys.argv[1]
mapAndDim = create_map(mapName)
p_safe = float(sys.argv[2])
configStr = f"{mapName}_p_safe_{p_safe}"
cache_successes = {}

mapsList = ["empty-32-32", "random-32-32-20", "maze-32-32-2", "room-32-32-4", "den312d", "ht_chantry", "lak303d",
            "den520d"]
num_of_agentsList = [10, 15, 20, 25, 30, 40, 50]
num_of_goalsList = [20, 30, 40, 50, 60, 80, 100]
no_collision_probList = [0.8, 0.9, 0.95]
delays_probList = [0.1, 0.3]
instances = 10
verifyAlpha = 0.05
max_time = 60

####################################################### Write the header of a CSV file ############################################################
if not os.path.exists("Output_files"):
    os.makedirs("Output_files")

with open(f"Output_files/Output_{configStr}.csv", mode="w", newline="",
          encoding="utf-8") as file:
    columns = ["Map", "Safe prob", "Delay prob", "Number of agents", "Number of goals", "Instance",
               "Runtime", "Offline runtime", "Online runtime", "Planner calls", "Sum Of Cost"]
    writer = csv.DictWriter(file, fieldnames=columns)
    writer.writeheader()


####################################################### Read locs from file #################################################################################
def read_locs_from_file(num_of_instance):
    file_agents_name = f"Agent_Goal_locations_files/{mapName.split('.')[0]}_Map_Agent_Locs_instance_{num_of_instance}.txt"
    file_goals_name = f"Agent_Goal_locations_files/{mapName.split('.')[0]}_Map_Goal_Locs_instance_{num_of_instance}.txt"

    with open(file_agents_name, "r") as f:
        Agents_Positions = [ast.literal_eval(line.strip()) for _, line in zip(range(CurrNumAgents), f)]

    with open(file_goals_name, "r") as f:
        Goals_Locations = [ast.literal_eval(line.strip()) for _, line in zip(range(CurrNumGoals), f)]

    return Agents_Positions, Goals_Locations


####################################################### run Test  #################################################################################
def run_Test(queue, AgentLocations, GoalLocations, DelaysProbDict):
    randGen = random.Random(44)
    OfflineTime, OnlineTime, callToPlanner, SOC = 0, 0, 0, 0
    while True:
        callToPlanner += 1
        offline_start_time = time.time()
        p = RobustCbssTuningDelays(AgentLocations, GoalLocations, p_safe, DelaysProbDict, mapAndDim, verifyAlpha)
        OfflineTime += (time.time() - offline_start_time)

        online_start_time = time.time()
        s = Run_Simulation(p.Solution[0], DelaysProbDict, AgentLocations, GoalLocations, randGen)
        OnlineTime += (time.time() - online_start_time)

        if s.runSimulation():
            SOC += s.SOC
            break
        Positions, GoalLocations = s.AgentLocations, s.remainGoals
        SOC += s.SOC

    queue.put((round(OfflineTime, 5), round(OnlineTime, 5), callToPlanner, SOC))

####################################################### run Tests #################################################################################

def run_instances():
    delaysProbDict = {i: CurrDelaysProb for i in range(CurrNumAgents)}

    for instance in range(instances):
        AgentsLocations, GoalsLocations = read_locs_from_file(instance)

        print(f"map: {mapName}, safe prob: {p_safe}, delay prob: {CurrDelaysProb}, agents: {CurrNumAgents}, goals: {CurrNumGoals}, instance: {instance}")
        print(f"AgentsPositions: {AgentsLocations} \n GoalsLocations: {GoalsLocations}")

        queue = Queue()

        process = Process(target=run_Test,
                          args=(queue, AgentsLocations, GoalsLocations, delaysProbDict))

        process.start()
        start_time = time.time()
        process.join(timeout=max_time)

        if process.is_alive():
            process.terminate()
            process.join()
            print(f"Skipped due to timeout after {max_time} seconds.")
            print("--------------------------------------------------------------------------")
            addRecordToCsv(instance, None, None, None, None, None)
            continue

        runtime = round(time.time() - start_time, 5)

        # If the process completed in time, retrieve the solution
        offlineRuntime, onlineRuntime, callToPlanner, soc = queue.get()
        print("Pass!")
        print("--------------------------------------------------------------------------")

        addRecordToCsv(instance, runtime, offlineRuntime, onlineRuntime, callToPlanner, soc)


def addRecordToCsv(instance, runtime, offlineRuntime, onlineRuntime, callToPlanner, soc):

    record = [mapName, p_safe, CurrDelaysProb, CurrNumAgents, CurrNumGoals, instance+1, runtime,
              offlineRuntime, onlineRuntime, callToPlanner, soc]

    with open(f"Output_{configStr}.csv", mode="a", newline="", encoding="utf-8") as file:
        writerRecord = csv.writer(file)
        writerRecord.writerow(record)


for CurrDelaysProb in delays_probList:
    for CurrNumAgents in num_of_agentsList:
        for CurrNumGoals in num_of_goalsList:
            run_instances()
