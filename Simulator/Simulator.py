import json
import random
import time
import tkinter as tk
import numpy as np

from Diagnosis.Brute_Force_Diagnosis import Brute_Force_Diagnosis
from Diagnosis.Greedy_Diagnosis import Greedy_Diagnosis
from Initialize_Simulator import initialize_simulation
from Visualization import initialize_visualize, visualize


def add_path_to_goal(agent, goal):
    path = []
    planned_path_of_agent = agent.get_planned_path()
    if len(planned_path_of_agent) != 0:
        x0, y0 = planned_path_of_agent[-1]
    else:
        x0, y0 = agent.get_location()
    x1, y1 = goal
    while x0 != x1:
        x0 += np.sign(x1 - x0)
        path.append([x0, y0])
    while y0 != y1:
        y0 += np.sign(y1 - y0)
        path.append([x0, y0])

    agent.add_to_planned_path(path)


def check_if_not_all_agents_finish(agents):
    for agent in agents:
        if len(agent.get_planned_path()) != 0:
            return True
    return False


def findFreeSlot(agent, grid, direction):
    x, y = -1, -1
    curr_loc = agent.get_location()

    if direction == "Vertical":
        if curr_loc[0] - 1 >= 0:
            if grid[curr_loc[0] - 1][curr_loc[1]] == 0:
                x, y = curr_loc[0] - 1, curr_loc[1]
        elif curr_loc[0] + 1 < len(grid):
            if grid[curr_loc[0] + 1][curr_loc[1]] == 0:
                x, y = curr_loc[0] + 1, curr_loc[1]

    elif direction == "Horizontal":
        if curr_loc[1] - 1 >= 0:
            if grid[curr_loc[0]][curr_loc[1] - 1] == 0:
                x, y = curr_loc[0], curr_loc[1] - 1
        elif curr_loc[1] + 1 < len(grid):
            if grid[curr_loc[0]][curr_loc[1] + 1] == 0:
                x, y = curr_loc[0], curr_loc[1] + 1

    else:
        return

    planned_path = agent.get_planned_path()
    planned_path.insert(0, curr_loc)
    grid[curr_loc[0]][curr_loc[1]] = 0
    grid[x][y] = agent
    agent.set_location([x, y])


def Try_Move(agent, grid, units_of_time, goals, agents):
    next_loc = agent.get_nextMove()
    if grid[next_loc[0]][next_loc[1]] != 0 and grid[next_loc[0]][next_loc[1]] in agents:
        if grid[next_loc[0]][next_loc[1]].get_nextMove() == agent.get_location():
            if agent.get_location()[0] == grid[next_loc[0]][next_loc[1]].get_location()[0]:
                findFreeSlot(agent, grid, "Vertical")
            else:
                findFreeSlot(agent, grid, "Horizontal")

    else:
        grid[agent.get_location()[0]][agent.get_location()[1]] = 0
        next_step = agent.move(units_of_time)
        grid[next_step[0]][next_step[1]] = agent
        if next_step in goals:
            grid[agent.get_location()[0]][agent.get_location()[1]] = 0
            goals.remove(next_step)

            if load_config('Simulator/config.json', "infinite") == "yes":
                random_goal = [random.randint(2, len(grid)-1), random.randint(2, len(grid)-1)]
                goals.append(random_goal)
                add_path_to_goal(agent, random_goal)


class SimulatorApp:
    def __init__(self, root, grid_size, loc_agents, loc_goals, delay_by_manufacturer,
                 failure_probability, time_to_diagnosis, DiagnosisAlg):

        self.root = root
        self.units_of_time = 0

        self.grid_size = grid_size
        self.goals = loc_goals
        self.time_to_diagnosis = time_to_diagnosis

        self.diagnosisAlg = DiagnosisAlg

        self.grid, self.agents = initialize_simulation(grid_size, loc_agents, loc_goals,
                                                       delay_by_manufacturer, failure_probability)

        initialize_visualize(self)

        for agent in self.agents:
            print(f"Delay by manufacturer A{agent.get_id()} {agent.get_delay_by_manufacturer()}")
        print("----------------------------------------------------------------")

        self.run_simulation()

    def run_simulation(self):

        if self.units_of_time % self.time_to_diagnosis == 0 and self.units_of_time != 0:
            start_time = time.time()
            self.agents, _ = self.diagnosisAlg.Find_Who_To_Repair_And_The_Remain_Agents(self.agents, self.goals)
            print(f"\nFinish time of the Diagnosis: {time.time() - start_time}")

        if check_if_not_all_agents_finish(self.agents):
            self.units_of_time += 1

            for agent in self.agents:
                if len(agent.get_planned_path()) != 0 and agent.checkIfCanMoveByDelay(self.units_of_time):
                    Try_Move(agent, self.grid, self.units_of_time, self.goals, self.agents)

            visualize(self)
            self.root.after(150, self.run_simulation)

        else:
            for agent in self.agents:
                print(f"Delay by observations A{agent.get_id()} {agent.get_delay_by_observations()}")



def load_config(filename, key="All"):
    with open(filename, 'r') as file:
        config = dict(json.load(file))

    if key == "All":

        if config["Diagnosis"] == "Brute_Force":
            config["Diagnosis"] = Brute_Force_Diagnosis(config["Replanning"])
        elif config["Diagnosis"] == "Greedy":
            config["Diagnosis"] = Greedy_Diagnosis(config["Replanning"])

        return config.values()

    return config[key]


if __name__ == "__main__":

    Root = tk.Tk()
    Root.title("Multi-Agent Simulator")

    (Grid_size, Loc_agents, Loc_goals, Delay_by_manufacturer,
     Failure_probability, Time_to_diagnosis, infinite, diagnosisAlg, _) = load_config('Simulator/config.json')

    SimulatorApp(Root, Grid_size, Loc_agents, Loc_goals, Delay_by_manufacturer,
                 Failure_probability, Time_to_diagnosis, diagnosisAlg)
    Root.mainloop()
