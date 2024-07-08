import numpy as np
from CBSS.libmcpf.cbss_msmp import RunCbssMSMP
from Agent import Agent


def bind_dest_to_agent(agents, goals):
    used_goals = set()
    destinations = []

    for agent in agents:
        max_distance = float('-inf')
        closet_goal = None
        for goal in goals:
            tuple_goal = tuple(goal)
            if tuple_goal not in used_goals:
                distance = abs(agent[0]-goal[0]) + abs(agent[1]+goal[1])
                if distance > max_distance:
                    max_distance = distance
                    closet_goal = goal

        used_goals.add(tuple(closet_goal))
        destinations.append(closet_goal)
    return destinations


def reduction_to_CBSS(loc_agents, loc_goals, grid_size):
    configs = dict()
    configs["problem_str"] = "msmp"
    configs["tsp_exe"] = "./CBSS/pytspbridge/tsp_solver/LKH-2.0.10/LKH"
    configs["time_limit"] = 1000
    configs["eps"] = 0.0

    grid = np.zeros((grid_size, grid_size))

    Loc_destinations = bind_dest_to_agent(loc_agents, loc_goals)
    loc_dest = [grid_size * y + x for x, y in Loc_destinations]
    loc_agents = [grid_size * y + x for x, y in loc_agents]
    loc_goals = [grid_size * y + x for x, y in loc_goals]

    return configs, grid, loc_agents, loc_goals, loc_dest


def generate_path(agents, loc_agents, loc_goals, grid_size):

    configs, grid, loc_agents, loc_goals, loc_destinations = (
        reduction_to_CBSS(loc_agents, loc_goals, grid_size))

    res_dict = RunCbssMSMP(grid, loc_agents, loc_goals, loc_destinations, configs)

    for key, value in res_dict["path_set"].items():
        path = []
        for x, y in list(zip(value[0], value[1]))[1:-1]:
            path.append([x, y])

        agents[key].set_planned_path(path)


def initialize_simulation(grid_size, loc_agents, loc_goals,
                          delay_by_manufacturer, failure_probability):
    agents = []
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

    for index, loc in enumerate(loc_agents):
        agent = Agent(delay_by_manufacturer=delay_by_manufacturer, location=loc,
                      failure_probability=failure_probability[index])

        grid[loc[0]][loc[1]] = agent
        agents.append(grid[loc[0]][loc[1]])

    generate_path(agents, loc_agents, loc_goals, grid_size)

    return grid, agents
