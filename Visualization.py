from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import tkinter as tk
import matplotlib.colors as mcolors

colors = list(mcolors.TABLEAU_COLORS.values())
agentsColors = {}


def initialize_visualize(SimulatorAppObj):
    SimulatorAppObj.fig, SimulatorAppObj.ax = plt.subplots(figsize=(6, 6))
    SimulatorAppObj.canvas = FigureCanvasTkAgg(SimulatorAppObj.fig, master=SimulatorAppObj.root)
    SimulatorAppObj.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)


def visualize(SimulatorAppObj, time_step):
    grid_size = len(SimulatorAppObj.grid)
    SimulatorAppObj.ax.clear()
    SimulatorAppObj.ax.set_xlim(-0.5, grid_size - 0.5)
    SimulatorAppObj.ax.set_ylim(-0.5, grid_size - 0.5)
    SimulatorAppObj.ax.set_xticks(np.arange(grid_size))
    SimulatorAppObj.ax.set_yticks(np.arange(grid_size))
    SimulatorAppObj.ax.set_xticks(np.arange(grid_size + 1) - 0.5, minor=True)
    SimulatorAppObj.ax.set_yticks(np.arange(grid_size + 1) - 0.5, minor=True)

    # Grid configuration
    SimulatorAppObj.ax.grid(which='minor', color='black', linestyle='-', linewidth=1)

    for agent in SimulatorAppObj.agents:
        agentsColors[agent.get_id()] = agentsColors.get(agent.get_id(), colors[agent.get_id() % len(colors)])
        # Plot goal
        for goal in agent.goals:
            SimulatorAppObj.ax.text(goal[1], goal[0], f"G{agent.get_id()}", fontsize=12, ha='center', va='center',
                                    color=agentsColors[agent.get_id()])
        # Plot agent
        location = agent.get_location()
        SimulatorAppObj.ax.text(location[1], location[0], f"A{agent.get_id()}", fontsize=12, ha='center', va='center',
                                color=agentsColors[agent.get_id()])
        # Plot agent's path
        # for step in agent.get_planned_path()[:-1]:
        #     SimulatorAppObj.ax.plot(step[1], step[0], '.', color=agentsColors[agent.get_id()], markersize=5)

    SimulatorAppObj.ax.set_title(f"Units Of Time: {time_step}", fontsize=16, loc='center')

    SimulatorAppObj.canvas.draw()