import heapq
import random
from itertools import combinations

def create_loc_times(path):
    locTimes = {}
    for i, loc in enumerate(path["path"]):
        locTimes.setdefault(loc, i)
    return locTimes


def create_edge_times(path):
    edgeTimes = {}
    for i in range(len(path["path"]) - 1):
        edge = (path["path"][i], path["path"][i + 1])
        edgeTimes.setdefault(edge, i + 1)
    return edgeTimes


class FindConflict:
    def __init__(self):
        self.randGen = random.Random(42)

    def findConflict(self, N):
        heap = []

        posConstraintsDict = {
            agent: {
                (x, (agent1, t1), (agent2, t2)) for agent1, agent2, x, t1, t2 in N.posConstraints[agent]
            }
            for agent in N.paths
        }

        agent_data = {
            agent: {
                "locTimes": create_loc_times(N.paths[agent]),
                "edgeTimes": create_edge_times(N.paths[agent])
            }
            for agent in N.paths
        }

        for agent1, agent2 in combinations(N.paths.keys(), 2):
            allPosConstDict = posConstraintsDict[agent1] | posConstraintsDict[agent2]

            locTimes1, edgeTimes1 = agent_data[agent1]["locTimes"], agent_data[agent1]["edgeTimes"]
            locTimes2, edgeTimes2 = agent_data[agent2]["locTimes"], agent_data[agent2]["edgeTimes"]

            for loc in locTimes1.keys() & locTimes2.keys():
                time1, time2 = locTimes1[loc], locTimes2[loc]
                delta = abs(time1 - time2)
                Time = min(time1, time2)

                agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

                if (loc, (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                    heapq.heappush(heap, (
                        delta, Time, self.randGen.random(), loc, (agent1, agent1_time), (agent2, agent2_time)))

            for edge1, time1 in edgeTimes1.items():
                reversed_edge1 = (edge1[1], edge1[0])
                if reversed_edge1 in edgeTimes2:
                    time2 = edgeTimes2[reversed_edge1]
                    delta = abs(time1 - time2)
                    Time = min(time1, time2)

                    agent1_time, agent2_time = (Time, Time + delta) if time1 <= time2 else (Time + delta, Time)

                    if (frozenset(edge1), (agent1, agent1_time), (agent2, agent2_time)) not in allPosConstDict:
                        heapq.heappush(heap, (
                            delta, Time, self.randGen.random(), frozenset(edge1), (agent1, agent1_time),
                            (agent2, agent2_time)))

        return heapq.heappop(heap) if heap else None
