from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from Simulate import initialize_simulation
import matplotlib.colors as mcolors


def visualize(agents, goals, grid_size, ax):
    ax.clear()
    ax.set_xlim(-1, grid_size)
    ax.set_ylim(-1, grid_size)

    for goal, agent in zip(goals, agents):
        # Plot goal
        ax.plot(goal[1], goal[0], 'o', color=agent.get_color(), markersize=15)

    for agent in agents:
        # Plot agent
        ax.plot(agent.get_location()[1], agent.get_location()[0], 'o', color=agent.get_color(), markersize=10)
        # Plot agent's path
        for step in agent.get_planned_path():
            ax.plot(step[1], step[0], '.', color=agent.get_color(), markersize=5)

    ax.grid(True)


class SimulatorApp:
    def __init__(self, root):
        self.root = root
        self.units_of_time = 0
        self.grid_size, self.num_agents, self.num_goals = 10, 6, 6

        self.agents, self.goals = initialize_simulation(self.grid_size, self.num_agents, self.num_goals)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        self.run_simulation()

    def run_simulation(self):
        if self.goals:
            self.units_of_time += 1
            for agent in self.agents:
                if len(agent.get_planned_path()) != 0 and self.units_of_time % agent.get_delay_by_manufacturer() == 0:
                    next_step_is_goal, next_step = agent.move()
                    if next_step_is_goal:
                        self.goals.remove(next_step)
            visualize(self.agents, self.goals, self.grid_size, self.ax)
            self.canvas.draw()
            self.root.after(200, self.run_simulation)
        else:
            print("Simulation completed in", self.units_of_time, "units of time")


if __name__ == "__main__":
    Root = tk.Tk()
    Root.title("Multi-Agent Pathfinding Simulator")
    app = SimulatorApp(Root)
    Root.mainloop()
