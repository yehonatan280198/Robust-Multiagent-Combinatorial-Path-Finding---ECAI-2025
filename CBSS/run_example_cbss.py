"""
Author: Zhongqiang (Richard) Ren
All Rights Reserved.
ABOUT: Entrypoint to the code.
Oeffentlich fuer: RSS22
"""

import context
import numpy as np
import cbss_msmp

def run_CBSS_MSMP():
  """
  fully anonymous case, no assignment constraints.
  """
  print("------run_CBSS_MSMP------")
  ny = 10
  nx = 10
  grids = np.zeros((ny,nx))
  # Image coordinate is used. Think about the matrix as a 2d image with the origin at the upper left corner.
  # Row index is y and column index is x.
  # For example, in the matrix, grids[3,4] means the vertex with coordinate y=3,x=4 (row index=3, col index=4).
  grids[5,3:7] = 1 # obstacles

  # The following are vertex IDs.
  # For a vertex v with (x,y) coordinate in a grid of size (Lx,Ly), the ID of v is y*Lx+x.
  starts = [1,3,5,7,9]
  targets = [40,38,27,66,72,81,83]
  dests = [91,93,95,97,99]

  configs = dict()
  configs["problem_str"] = "msmp"
  configs["tsp_exe"] = "./pytspbridge/tsp_solver/LKH-2.0.10/LKH"
  configs["time_limit"] = 60
  configs["eps"] = 0.0
  res_dict = cbss_msmp.RunCbssMSMP(grids, starts, targets, dests, configs)
  
  print(res_dict)

  return


if __name__ == '__main__':

  run_CBSS_MSMP()

