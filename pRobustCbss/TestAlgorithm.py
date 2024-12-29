import random

from collections import defaultdict
from multiprocessing import Process, Queue

from pRobustCbss.Run_pRobustCbss import pRobustCbss

def run_pRobustCbss(queue, Positions, GoalLocations, no_collision_prob, delaysProb, rows, cols, verifyAlpha, OriginalCheckRoot, rotate, prob):
    """Wrap the pRobustCbss execution in a separate process."""

    p = pRobustCbss(Positions, GoalLocations, no_collision_prob, delaysProb, rows, cols, verifyAlpha, OriginalCheckRoot, rotate, prob)
    queue.put(dict(p.Solution))


# cases = [[True, False, False], [False, False, False], [False, True, False], [False, False, True], [False, True, True]]
cases = [[False, True, False], [False, True, True]]

size_of_grid = [20, 20]
num_of_agentsList = [3, 5, 5, 10, 15, 20]
num_of_goalsList = [5, 7, 10, 20, 30, 50]
no_collision_prob = 0.6
verifyAlpha = 0.05
samples = 5
max_time = 60

dict_of_successes = defaultdict(int)
for iteration in range(6):
    num_of_agents = num_of_agentsList[iteration]
    num_of_goals = num_of_goalsList[iteration]
    delaysProb = {i: 0.1 for i in range(num_of_agents)}

    for sample in range(samples):
        Positions = [(num, random.randint(0, 3)) for num in
                     random.sample(range(size_of_grid[0] * size_of_grid[1]), num_of_agents)]

        excluded_numbers = {t[0] for t in Positions}
        available_numbers = [num for num in range(size_of_grid[0] * size_of_grid[1]) if num not in excluded_numbers]
        GoalLocations = random.sample(available_numbers, num_of_goals)

        print("Positions:", Positions)
        print("GoalLocations:", GoalLocations)

        for i, case in enumerate(cases):
            successful_runs = True

            queue = Queue()
            process = Process(target=run_pRobustCbss, args=(
                queue, Positions, GoalLocations, no_collision_prob, delaysProb, size_of_grid[0], size_of_grid[1], verifyAlpha, case[0], case[1], case[2]))

            process.start()
            process.join(timeout=max_time)

            if process.is_alive():
                process.terminate()
                process.join()
                print(f"Skipped due to timeout after {max_time} seconds.")
                print("--------------------------------------------------------------------------")
                successful_runs = False
                continue

            # If the process completed in time, retrieve the solution
            solution = queue.get()
            print("Solution:", solution)
            print("--------------------------------------------------------------------------")

            if successful_runs:
                dict_of_successes[(i, num_of_agents, num_of_goals)] += 1

    print(dict_of_successes)


