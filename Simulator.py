import tkinter as tk
from Initialize_Simulator import initialize_simulation
from Visualization import initialize_visualize, visualize


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
    agent.set_location((x, y))


def Try_Move(agent, goals, grid, units_of_time):
    next_loc = agent.get_nextMove()

    if grid[next_loc[0]][next_loc[1]] != 0:
        if grid[next_loc[0]][next_loc[1]].get_nextMove() == agent.get_location():
            if agent.get_location()[0] == grid[next_loc[0]][next_loc[1]].get_location()[0]:
                findFreeSlot(agent, grid, "Vertical")
            else:
                findFreeSlot(agent, grid, "Horizontal")

    else:
        grid[agent.get_location()[0]][agent.get_location()[1]] = 0
        next_step_is_goal, next_step = agent.move(units_of_time)
        grid[next_step[0]][next_step[1]] = agent
        if next_step_is_goal:
            grid[agent.get_location()[0]][agent.get_location()[1]] = 0
            goals.remove(next_step)


class SimulatorApp:
    def __init__(self, root, grid_size, num_agents, num_goals):
        self.root = root
        self.grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.units_of_time = 0
        self.agents, self.goals = initialize_simulation(num_agents, num_goals, self.grid)
        initialize_visualize(self)

        for agent in self.agents:
            print(f"Delay by manufacturer A{agent.get_id()} {agent.get_delay_by_manufacturer()}")
        print("----------------------------------------------------------------")
        self.run_simulation()

    def run_simulation(self):
        if self.goals:
            self.units_of_time += 1
            for agent in self.agents:
                if len(agent.get_planned_path()) != 0 and self.units_of_time % agent.get_delay_by_manufacturer() == 0:
                    Try_Move(agent, self.goals, self.grid, self.units_of_time)

            visualize(self, self.units_of_time)

        else:
            for agent in self.agents:
                print(f"Delay by observations A{agent.get_id()} {agent.get_delay_by_observations()}")


if __name__ == "__main__":
    Root = tk.Tk()
    Root.title("Multi-Agent Simulator")
    SimulatorApp(Root, 10, 10, 10)
    Root.mainloop()
