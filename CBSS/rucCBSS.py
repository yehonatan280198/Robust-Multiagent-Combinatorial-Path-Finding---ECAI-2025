"""
Author: Zhongqiang (Richard) Ren
All Rights Reserved.
ABOUT: Entrypoint to the code.
Oeffentlich fuer: RSS22
"""
import os

import numpy as np
from CBSS.libmcpf import cbss_msmp


def run_CBSS_MSMP(size_grid, locations, taskLocs, delays):
    current_dir = os.getcwd()
    os.chdir('/home/yonikid/Desktop/SimulatorAgents/CBSS')

    print("------run_CBSS_MSMP------")

    grids = np.zeros((size_grid, size_grid))
    starts = locations
    targets = taskLocs

    configs = dict()
    configs["problem_str"] = "msmp"
    configs["tsp_exe"] = "./pytspbridge/tsp_solver/LKH-2.0.10/LKH"
    configs["time_limit"] = 600
    configs["eps"] = 0.0
    res_dict = cbss_msmp.RunCbssMSMP(grids, starts, targets, configs, delays)

    os.chdir(current_dir)

    paths = {}
    makeSpan = float('-inf')
    for key, value in res_dict["path_set"].items():
        if len (value[2]) == 0:
            paths[key] = []
            continue

        if makeSpan < value[2][-2]:
            makeSpan = value[2][-2]

        path = []
        for x, y in list(zip(value[0], value[1]))[1:-1]:
            path.append(y*size_grid+x)

        paths[key] = path

    return paths, makeSpan, res_dict["assignGoals"]
