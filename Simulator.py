import tkinter as tk
from Initialize_Simulator import initialize_simulation
from visualization import initialize_visualize, visualize


def Try_Move(agent, goals, taken_slots):
    if agent.get_nextMove() in taken_slots:
        return False

    else:
        taken_slots.remove(agent.get_location())
        next_step_is_goal, next_step = agent.move()
        taken_slots.add(next_step)
        if next_step_is_goal:
            taken_slots.remove(agent.get_location())
            goals.remove(next_step)

    return True


class SimulatorApp:
    def __init__(self, root):
        self.root = root
        self.units_of_time = 0
        self.grid_size, self.num_agents, self.num_goals = 10, 6, 6
        self.taken_slots = set()

        self.agents, self.goals = initialize_simulation(self.grid_size, self.num_agents, self.num_goals, self.taken_slots)

        initialize_visualize(self)

        self.run_simulation()

    def run_simulation(self):
        if self.goals:
            self.units_of_time += 1
            for agent in self.agents:
                if len(agent.get_planned_path()) != 0 and self.units_of_time % agent.get_delay_by_manufacturer() == 0:
                    Try_Move(agent, self.goals, self.taken_slots)

            visualize(self)

        else:
            print("Simulation completed in", self.units_of_time, "units of time")


if __name__ == "__main__":
    Root = tk.Tk()
    Root.title("Multi-Agent Simulator")
    SimulatorApp(Root)
    Root.mainloop()
