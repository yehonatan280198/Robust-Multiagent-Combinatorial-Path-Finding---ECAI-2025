import random
from random import randint
from scipy.optimize import linear_sum_assignment
import numpy as np
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


def initialize_simulation(num_agents, num_goals, grid):
    goals, init_location, agents = [], [], []

    while len(goals) < num_goals:
        location = (randint(4, len(grid) - 1), randint(0, len(grid) - 1))
        if location not in goals:
            goals.append(location)

    while len(init_location) < num_agents:
        location = (0, randint(0, len(grid) - 1))
        if location not in init_location:
            init_location.append(location)
            agents.append(Agent(delay_by_manufacturer=randint(1, 3), location=location,
                                failure_probability=random.random()))
            grid[location[0]][location[1]] = agents[-1]

    agent_indices, goal_indices = Solve_The_Assignment_Problem(agents, goals)

    for agent_index, goal_index in zip(agent_indices, goal_indices):
    #for agent_index, goal_index in zip(range(0, 10), range(0, 10)):
        agents[agent_index].add_goal(goals[goal_index])
        generate_path(agents[agent_index], goals[goal_index])

    return agents, goals
