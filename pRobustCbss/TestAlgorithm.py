import random
import time
from multiprocessing import Process, Queue

from pRobustCbss.Run_pRobustCbss import pRobustCbss

size_of_grid = [12, 12]
num_of_agents = 5
num_of_goals = 7
no_collision_prob = 0.1
delaysProb = {i: 0.1 for i in range(num_of_agents)}
verifyAlpha = 0.05
samples = 50
max_time = 60

def run_pRobustCbss(queue, Positions, GoalLocations, no_collision_prob, delaysProb, rows, cols, verifyAlpha):
    """Wrap the pRobustCbss execution in a separate process."""
    try:
        p = pRobustCbss(Positions, GoalLocations, no_collision_prob, delaysProb, rows, cols, verifyAlpha)
        queue.put(dict(p.Solution))
    except Exception as e:
        queue.put(f"Error: {str(e)}")


successful_runs = 0

for sample in range(samples):
    Positions = [(num, random.randint(0, 3)) for num in
                 random.sample(range(size_of_grid[0] * size_of_grid[1]), num_of_agents)]

    excluded_numbers = {t[0] for t in Positions}
    available_numbers = [num for num in range(size_of_grid[0] * size_of_grid[1]) if num not in excluded_numbers]
    GoalLocations = random.sample(available_numbers, num_of_goals)

    print("Positions:", Positions)
    print("GoalLocations:", GoalLocations)

    queue = Queue()
    process = Process(target=run_pRobustCbss, args=(
        queue, Positions, GoalLocations, no_collision_prob, delaysProb, size_of_grid[0], size_of_grid[1], verifyAlpha))

    process.start()
    process.join(timeout=max_time)

    if process.is_alive():
        process.terminate()
        process.join()
        print(f"Skipped due to timeout after {max_time} seconds.")
        print("--------------------------------------------------------------------------")
        continue

    # If the process completed in time, retrieve the solution
    solution = queue.get()
    successful_runs += 1
    print("Solution:", solution)
    print("--------------------------------------------------------------------------")


print(f"Number of successful runs: {successful_runs} out of {samples}")
