from random import randint
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import matplotlib.colors as mcolors
import numpy as np
from scipy.optimize import linear_sum_assignment

from Agent import Agent


def Solve_The_Assignment_Problem(agents, goals):
    """Solves the assignment problem optimally using the Hungarian algorithm."""
    cost_matrix = np.zeros((len(agents), len(goals)))
    for i, agent in enumerate(agents):
        for j, goal in enumerate(goals):
            cost_matrix[i, j] = abs(agent.get_location()[0] - goal[0]) + abs(agent.get_location()[1] - goal[1])
    return linear_sum_assignment(cost_matrix)


def generate_path(agent, goal):
    """Generates a path from start to end, allowing only horizontal and vertical movements."""
    path = []
    x0, y0 = agent.get_location()
    x1, y1 = goal
    while x0 != x1:
        x0 += np.sign(x1 - x0)
        path.append((x0, y0))
    while y0 != y1:
        y0 += np.sign(y1 - y0)
        path.append((x0, y0))
    agent.set_planned_path(path)


def initialize_simulation(grid_size, num_agents, num_goals):
    colors = list(mcolors.TABLEAU_COLORS.values())
    goals, init_location, agents = [], [], []

    while len(goals) < num_goals:
        location = (randint(4, grid_size - 1), randint(0, grid_size - 1))
        if location not in goals:
            goals.append(location)

    while len(init_location) < num_agents:
        location = (0, randint(0, grid_size - 1))
        if location not in init_location:
            init_location.append(location)
            agents.append(Agent(delay_by_manufacturer=randint(1, 3), location=location, color=colors[len(init_location) % len(colors)], agent_id=len(init_location)))

    agent_indices, goal_indices = Solve_The_Assignment_Problem(agents, goals)

    for agent_index, goal_index in zip(agent_indices, goal_indices):
        agents[agent_index].add_goal(goals[goal_index])
        generate_path(agents[agent_index], goals[goal_index])

    return agents, goals


def initialize_visualize(SimulatorAppObj):
    SimulatorAppObj.fig, SimulatorAppObj.ax = plt.subplots(figsize=(5, 5))
    SimulatorAppObj.canvas = FigureCanvasTkAgg(SimulatorAppObj.fig, master=SimulatorAppObj.root)
    SimulatorAppObj.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)


def visualize(SimulatorAppObj):
    SimulatorAppObj.ax.clear()
    SimulatorAppObj.ax.set_xlim(-1, SimulatorAppObj.grid_size)
    SimulatorAppObj.ax.set_ylim(-1, SimulatorAppObj.grid_size)

    for agent in SimulatorAppObj.agents:
        # Plot goal
        for goal in agent.goals:
            SimulatorAppObj.ax.text(goal[1], goal[0], f"G{agent.get_id()}", fontsize=12, ha='center', va='center', color=agent.get_color())
        # Plot agent
        location = agent.get_location()
        SimulatorAppObj.ax.text(location[1], location[0], f"A{agent.get_id()}", fontsize=12, ha='center', va='center', color=agent.get_color())
        # Plot agent's path
        for step in agent.get_planned_path()[:-1]:
            SimulatorAppObj.ax.plot(step[1], step[0], '.', color=agent.get_color(), markersize=5)

    SimulatorAppObj.ax.grid(True)
    SimulatorAppObj.canvas.draw()
    SimulatorAppObj.root.after(200, SimulatorAppObj.run_simulation)



class SimulatorApp:
    def __init__(self, root):
        self.root = root
        self.units_of_time = 0
        self.grid_size, self.num_agents, self.num_goals = 10, 6, 6

        self.agents, self.goals = initialize_simulation(self.grid_size, self.num_agents, self.num_goals)

        initialize_visualize(self)

        self.run_simulation()

    def run_simulation(self):
        if self.goals:
            self.units_of_time += 1
            for agent in self.agents:
                if len(agent.get_planned_path()) != 0 and self.units_of_time % agent.get_delay_by_manufacturer() == 0:
                    next_step_is_goal, next_step = agent.move()
                    if next_step_is_goal:
                        self.goals.remove(next_step)

            visualize(self)

        else:
            print("Simulation completed in", self.units_of_time, "units of time")


if __name__ == "__main__":
    Root = tk.Tk()
    Root.title("Multi-Agent Pathfinding Simulator")
    app = SimulatorApp(Root)
    Root.mainloop()
