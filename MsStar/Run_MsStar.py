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
    for i, (agent_loc, _) in enumerate(Vl):
        if agent_loc in location_counts:
            agent_conflict.update([i, location_counts[agent_loc]])
        else:
            location_counts[agent_loc] = i

    # Edge conflicts
    agent_loc_dict = {loc: idx for idx, (loc, _) in enumerate(Vl)}
    for i, (agent_loc, _) in enumerate(Vl):
        if Vk[i] != agent_loc and Vk[i] in agent_loc_dict:
            index = agent_loc_dict[Vk[i]]
            if agent_loc == Vk[index]:
                agent_conflict.update([i, index])

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
        h_val, task_allocation = self.calc_heuristic_value2(S0)
        self.OPEN.put((h_val + 0, S0))

        self.compute_individual_optimal_policy(task_allocation, initial_locations)

    def run(self):
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
                F_val = Sl.g + self.calc_heuristic_value2(Sl)[0]

                dominated_state = Check_who_is_dominant(Sl, self.Alpha[Sl.Vk])

                # if sl is non-dominated by any states in α(vl) then
                if not dominated_state:
                    self.OPEN.put((F_val, Sl))  # add sl to OPEN
                    Sl.parent = Sk  # parent(sl) ← sk
                    self.Alpha[Sl.Vk].add(Sl)

                # else (Notes: sl is dominated by states in α(vl))
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
        return heuristic_makespan

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
                    next_optimal_move = self.optimal_policy[i][min(self.optimal_policy[i].index((agent_loc, agent_direction)) + 1,len(self.optimal_policy[i])-1)]
                    vi.append(next_optimal_move)
                    if next_optimal_move[0] in self.taskLocs:
                        ai[self.taskLocs.index(next_optimal_move[0])] = 1

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
                F_val = s.g + self.calc_heuristic_value(s)
                self.OPEN.put((F_val, s))
            for Sk in s.back_set:
                self.BackProp(Sk, s.Ic)

    def calc_heuristic_value2(self, s):
        makespan = []
        goal_permutations = list(permutations(self.taskLocs))

        for perm in goal_permutations:
            agent_allocations = product(range(0,len(s.Vk)), repeat=len(perm))

            for allocation in agent_allocations:
                combination = list(zip(perm, allocation))
                goal_locations = [[loc] for loc, _ in s.Vk]

                for goal, agent in combination:
                    goal_locations[agent].append(goal)

                time_to_finish = []

                for goals in goal_locations:
                    time = 0
                    for i in range(len(goals) - 1):
                        time = (time + abs(goals[i] // self.num_cols - goals[i + 1] // self.num_cols) +
                                abs(goals[i] % self.num_cols - goals[i + 1] % self.num_cols))

                    if len(goals) != 1:
                        time_to_finish.append((time, goal_locations))

                makespan.append(max(time_to_finish))

        return min(makespan)

    def compute_individual_optimal_policy(self, task_allocation, initial_location):
        for agent, goals_per_agent in enumerate(task_allocation):
            self.optimal_policy[agent] = [initial_location[agent]]

            if len(goals_per_agent) != 1:
                for i in range(len(goals_per_agent) - 1):
                    curr_loc = list(self.optimal_policy[agent][-1])
                    next_goal = goals_per_agent[i + 1]

                    up_or_down = -1
                    left_or_right = -1

                    if goals_per_agent[i] // self.num_cols > next_goal // self.num_cols:
                        up_or_down = 3
                    if goals_per_agent[i] // self.num_cols < next_goal // self.num_cols:
                        up_or_down = 1
                    if goals_per_agent[i] % self.num_cols > next_goal % self.num_cols:
                        left_or_right = 2
                    if goals_per_agent[i] % self.num_cols < next_goal % self.num_cols:
                        left_or_right = 0

                    while curr_loc[0] != next_goal:
                        if curr_loc[1] == up_or_down == 1:
                            while (curr_loc[0] + self.num_cols) // self.num_cols <= (goals_per_agent[i + 1]) // self.num_cols:
                                curr_loc[0] += self.num_cols
                                self.optimal_policy[agent].append(tuple(curr_loc))

                        elif curr_loc[1] == up_or_down == 3:
                            while (curr_loc[0] - self.num_cols) // self.num_cols >= (goals_per_agent[i + 1]) // self.num_cols:
                                curr_loc[0] -= self.num_cols
                                self.optimal_policy[agent].append(tuple(curr_loc))

                        elif curr_loc[1] == left_or_right == 0:
                            while (curr_loc[0] + 1) % self.num_cols <= goals_per_agent[i + 1] % self.num_cols:
                                curr_loc[0] += 1
                                self.optimal_policy[agent].append(tuple(curr_loc))

                        elif curr_loc[1] == left_or_right == 2:
                            while (curr_loc[0] - 1) % self.num_cols >= goals_per_agent[i + 1] % self.num_cols:
                                curr_loc[0] -= 1
                                self.optimal_policy[agent].append(tuple(curr_loc))

                        if curr_loc[0] != next_goal:
                            if (curr_loc[1] + 1) % 4 == up_or_down or (curr_loc[1] + 1) % 4 == left_or_right:
                                curr_loc[1] = (curr_loc[1] + 1) % 4
                            elif (curr_loc[1] - 1) % 4 == up_or_down or (curr_loc[1] - 1) % 4 == left_or_right:
                                curr_loc[1] = (curr_loc[1] - 1) % 4
                            self.optimal_policy[agent].append(tuple(curr_loc))

