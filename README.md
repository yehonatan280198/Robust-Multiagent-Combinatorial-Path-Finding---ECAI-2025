# Robust Multiagent Combinatorial Path Finding

This repository implements the core algorithms and methods described in the paper "Robust Multi-Agent Combinatorial Path Finding".

## Files overview

* FindConflict.py - Implements the conflict detection logic used by the high-level CBS framework. Identifies both standard and delay-based conflicts between agents' paths.
* LowLevelPlan.py - Contains the low-level planner responsible for computing individual agent paths given a set of constraints and a fixed goal sequence allocation.
* NodeStateConstClasses.py - Defines the data structures for representing constraint tree nodes, constraints, and agent states used throughout the planning process.
* Run_pRobustCbss.py - Main entry point for running the pRobustCbss algorithm. Handles initialization, parameter setup, and execution of the full planning pipeline.
* Simulation_for_type2_test.py - A simulation script designed to test the planner’s performance under specific configurations (Type 2 tests) for robustness evaluation.
* TestVsBaseline1.py - Runs comparative experiments between pRobustCbss and the first baseline method, allowing benchmark analysis and result collection.
* TestVsBaseline2And3.py - Similar to TestVsBaseline1.py, this script compares pRobustCbss with Baseline 2 and Baseline 3, supporting extended benchmarking.
* Verify.py - Provides verification utilities for checking the correctness and robustness of computed solutions, including probabilistic robustness validation.
* createMap.py - Utility script to create and initialize test maps/environments used in simulations, including agent and goal placements.
* kBestSequencing.py - Implements the core logic of the K-Best Sequencing procedure, generating multiple optimal goal sequence allocations by solving the problem as a standard TSP after applying a transformation.
* kBestSequencingWithGLKH.py - A variant of kBestSequencing.py that directly integrates the Generalized Lin-Kernighan-Helsgaun (GLKH) solver to solve the problem as an E-GTSP, allowing native handling of clusters and improving accuracy for complex goal structures.

## Directories Overview

* ATSP_runtime_files - Contains temporary files and runtime data generated during the execution of the ATSP (Asymmetric Traveling Salesman Problem) solver. These files are used internally for solver configuration and result parsing.
* Agent_Goal_locations_files - Stores input files defining the initial configurations of agents and their corresponding goal locations. This folder provides the scenario setup for each experimental run.
* EGSTP_runtime_files - Contains runtime files specific to the E-GTSP (Equality Generalized Traveling Salesman Problem) solver. These files handle the transformed problem representations and solver outputs.
* GLKH-1.1 - The source code and binaries of the GLKH solver (version 1.1), which extends the LKH framework to handle E-GTSP instances directly. Used by the kBestSequencingWithGLKH.py module.
* LKH-3.0.11 - Contains the standard LKH solver (version 3.0.11) for solving classical TSP and ATSP instances. This solver is integrated into the kBestSequencing.py module to generate goal sequence allocations based on TSP formulations, and is also used for baseline comparisons in the planning pipeline.
* OurResearch.domain - Contains the map files used in the experiments. Each file defines a specific environment (grid) for testing and evaluating the algorithms.
