from itertools import permutations, product

from collections import defaultdict
from queue import PriorityQueue

from MsStar.State import State


def Reconstruct(Sk):
    paths = []
    while Sk is not None:
        paths.append(Sk.Vk)
        Sk = Sk.parent
    paths.reverse()
    return list(zip(*paths))


def Check_who_is_dominant(Sl, states):
    dominant_states = set()
    for s in states:
        if not Sl.more_dominant_than(s):
            dominant_states.add(s)
    return dominant_states


def find_collision(Vk, Vl):
    agent_conflict = set()

    # Vertex conflicts
    location_counts = {}
    for agent, (agent_loc, _) in enumerate(Vl):
        if agent_loc in location_counts:
            agent_conflict.update([agent, location_counts[agent_loc]])
        else:
            location_counts[agent_loc] = agent

    # Edge conflicts
    agent_curr_loc_dict = {agent_loc: agent for agent, (agent_loc, _) in enumerate(Vl)}
    for agent, (agent_curr_loc, _) in enumerate(Vl):
        previous_loc = Vk[agent]
        if previous_loc != agent_curr_loc and previous_loc in agent_curr_loc_dict:
            other_agent = agent_curr_loc_dict[previous_loc]
            if agent_curr_loc == Vk[other_agent]:
                agent_conflict.update([agent, other_agent])

    return agent_conflict


class MsStar:
    def __init__(self, initial_locations, taskLocs, num_cols, num_rows):
        self.taskLocs = taskLocs
        self.Alpha = defaultdict(set)
        self.OPEN = PriorityQueue()
        self.num_cols = num_cols
        self.num_rows = num_rows
        self.optimal_policy = {}

        # Initialize OPEN with so = (vo, 0^M)
        S0 = State(initial_locations, [0] * len(taskLocs))
        h_val, task_allocation = self.Heuristics_for_makespan_and_task_order_allocation(S0)
        self.OPEN.put((h_val + 0, S0))

        print(task_allocation)
        self.compute_individual_optimal_policy(task_allocation, initial_locations)

    def run(self):
        print(self.optimal_policy)
        # While OPEN not empty do
        while not self.OPEN.empty():

            # sk = (vk, ak) ← OPEN.pop()
            f, Sk = self.OPEN.get()

            # if sk ∈ Sf then
            if list(Sk.Ak) == [1] * len(self.taskLocs):
                paths = Reconstruct(Sk)  # π ← Reconstruct(sk)
                return paths

            # S-ngh ← GetNeighbor(sk)
            Sngh = self.GetNeighbor(Sk)

            # for all sl = (vl, al) ∈ S-ngh do
            for Sl in Sngh:
                Sl.back_set.add(Sk)  # add sk to back set(sl)
                set_collision = find_collision(Sk.Vk, Sl.Vk)

                # if Ψ(vk, vl) != ∅
                if set_collision:
                    Sl.Ic.update(set_collision)  # IC (sl) ← IC (sl) ∪ Ψ(vk, vl)
                    self.BackProp(Sk, Sl.Ic)  # BackProp(sk, IC (sl))
                    continue

                # f(sl) ← g(sl) + h(sl)
                Sl.g = Sk.g + 1
                F_val = Sl.g + self.Heuristics_for_makespan_and_task_order_allocation(Sl)[0]

                dominated_state = Check_who_is_dominant(Sl, self.Alpha[Sl.Vk])

                # if sl is non-dominated by any states in α(vl) then
                if not dominated_state:
                    self.OPEN.put((F_val, Sl))  # add sl to OPEN
                    Sl.parent = Sk  # parent(sl) ← sk
                    self.Alpha[Sl.Vk].add(Sl)

                # else (sl is dominated by states in α(vl))
                else:

                    # for all s'l ∈ A do
                    for SlTag in dominated_state:
                        self.BackProp(Sk, SlTag.Ic)  # BackProp(sk, IC (s'l))
                        SlTag.back_set.add(Sk)  # add sk to back set(s'l)

        return []

    def calc_heuristic_value(self, s):
        min_longest_paths = []

        for agent_loc, agent_direct in s.Vk:
            distances = [
                abs(agent_loc // self.num_cols - self.taskLocs[i] // self.num_cols) +
                abs(agent_loc % self.num_cols - self.taskLocs[i] % self.num_cols)
                for i, val in enumerate(s.Ak) if val == 0
            ]
            min_longest_paths.append(min(distances, default=0))

        heuristic_makespan = max(min_longest_paths)
        return heuristic_makespan, []

    def validateMove(self, loc):
        return not (loc // self.num_cols >= self.num_rows or loc % self.num_cols >= self.num_cols)

    def GetNeighbor(self, Sk):
        neighbors = set()
        ranges = [range(4)] * len(Sk.Vk)

        for combination in product(*ranges):
            vi = []
            ai = list(Sk.Ak)
            flag = True

            for i, (agent_loc, agent_direction) in enumerate(Sk.Vk):
                action = combination[i]

                if i not in Sk.Ic:
                    next_optimal_move = self.optimal_policy[i][
                        min(self.optimal_policy[i].index((agent_loc, agent_direction)) + 1,
                            len(self.optimal_policy[i]) - 1)]
                    vi.append(next_optimal_move)
                    if next_optimal_move[0] in self.taskLocs:
                        ai[self.taskLocs.index(next_optimal_move[0])] = 1

                # if 1 == 0:
                #     pass

                elif action == 0:
                    candidates = [agent_loc + 1, agent_loc + self.num_cols, agent_loc - 1, agent_loc - self.num_cols]
                    loc_after_move = candidates[agent_direction]

                    if 0 <= loc_after_move < self.num_cols * self.num_rows and self.validateMove(loc_after_move):
                        vi.append((loc_after_move, agent_direction))
                        if loc_after_move in self.taskLocs:
                            ai[self.taskLocs.index(loc_after_move)] = 1

                    else:
                        flag = False
                        break

                elif action == 1:
                    new_direction = (agent_direction - 1) % 4
                    vi.append((agent_loc, new_direction))

                elif action == 2:
                    new_direction = (agent_direction + 1) % 4
                    vi.append((agent_loc, new_direction))

                else:
                    vi.append((agent_loc, agent_direction))

            if flag:
                neighbors.add(State(vi, tuple(ai)))

        return neighbors

    def BackProp(self, s, I):
        if not I.issubset(s.Ic):
            s.Ic.update(I)
            if s not in {item[1] for item in self.OPEN.queue}:
                F_val = s.g + self.Heuristics_for_makespan_and_task_order_allocation(s)[0]
                self.OPEN.put((F_val, s))
            for Sk in s.back_set:
                self.BackProp(Sk, s.Ic)

    def Heuristics_for_makespan_and_task_order_allocation(self, s):
        makespan_per_combination = []
        uncollected_tasks = [task for index, task in enumerate(self.taskLocs) if s.Ak[index] == 0]

        goals_permutations = list(permutations(uncollected_tasks))
        for goals_perm in goals_permutations:
            agents_allocations = product(range(0, len(s.Vk)), repeat=len(goals_perm))

            for agents_alloc in agents_allocations:
                combination = list(zip(goals_perm, agents_alloc))
                goal_locations = [[[loc, direction]] for (loc, direction) in s.Vk]

                for goal, agent in combination:
                    goal_locations[agent].append([goal, -1])

                finish_time_of_each_agent = []

                for goals in goal_locations:
                    total_time = 0
                    for i in range(len(goals) - 1):
                        time, final_direction = self.compute_minimal_time_to_reach_goal_and_final_direction(goals[i], goals[i + 1])
                        total_time = total_time + time
                        goals[i + 1][1] = final_direction

                    finish_time_of_each_agent.append((total_time, [[goal[0] for goal in goals] for goals in goal_locations]))

                makespan_per_combination.append(max(finish_time_of_each_agent))

        return min(makespan_per_combination)

    def compute_individual_optimal_policy(self, task_allocation, initial_location):
        for agent, goals_per_agent in enumerate(task_allocation):
            self.optimal_policy[agent] = [initial_location[agent]]

            for i in range(len(goals_per_agent) - 1):
                curr_loc, curr_direct = list(self.optimal_policy[agent][-1])
                next_goal = goals_per_agent[i + 1]

                curr_row = curr_loc // self.num_cols
                curr_col = curr_loc % self.num_cols
                next_goal_row = next_goal // self.num_cols
                next_goal_col = next_goal % self.num_cols

                up_or_down = 3 if curr_row > next_goal_row else (1 if curr_row < next_goal_row else None)
                left_or_right = 2 if curr_col > next_goal_col else (0 if curr_col < next_goal_col else None)

                while curr_loc != next_goal:

                    if curr_direct == up_or_down == 1:
                        while (curr_loc + self.num_cols) // self.num_cols <= next_goal_row:
                            curr_loc += self.num_cols
                            self.optimal_policy[agent].append(tuple((curr_loc, curr_direct)))

                    elif curr_direct == up_or_down == 3:
                        while (curr_loc - self.num_cols) // self.num_cols >= next_goal_row:
                            curr_loc -= self.num_cols
                            self.optimal_policy[agent].append(tuple((curr_loc, curr_direct)))

                    elif curr_direct == left_or_right == 0:
                        while (curr_loc + 1) % self.num_cols <= next_goal_col:
                            curr_loc += 1
                            self.optimal_policy[agent].append(tuple((curr_loc, curr_direct)))

                    elif curr_direct == left_or_right == 2:
                        while (curr_loc - 1) % self.num_cols >= next_goal_col:
                            curr_loc -= 1
                            self.optimal_policy[agent].append(tuple((curr_loc, curr_direct)))

                    if curr_loc != next_goal:
                        if (curr_direct + 1) % 4 == up_or_down or (curr_direct + 1) % 4 == left_or_right:
                            curr_direct = (curr_direct + 1) % 4
                        elif (curr_direct - 1) % 4 == up_or_down or (curr_direct - 1) % 4 == left_or_right:
                            curr_direct = (curr_direct - 1) % 4
                        self.optimal_policy[agent].append(tuple((curr_loc, curr_direct)))

    def compute_minimal_time_to_reach_goal_and_final_direction(self, curr, goal):
        curr_row = curr[0] // self.num_cols
        curr_col = curr[0] % self.num_cols
        goal_row = goal[0] // self.num_cols
        goal_col = goal[0] % self.num_cols

        time = abs(curr_row - goal_row) + abs(curr_col - goal_col)

        up_or_down = 3 if curr_row > goal_row else (1 if curr_row < goal_row else None)
        left_or_right = 2 if curr_col > goal_col else (0 if curr_col < goal_col else None)

        direction = -1

        if curr[1] == up_or_down or curr[1] == left_or_right:
            if curr_row != goal_row and curr_col != goal_col:
                time += 1
            direction = left_or_right if curr[1] == up_or_down else up_or_down

        else:
            time += 2
            direction = left_or_right if curr[1] in [1, 3] else up_or_down

        return time, direction
