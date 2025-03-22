import numpy as np
import os
import subprocess
from datetime import datetime

from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot
from ompl import util as ou

from drake_ompl.setup_scene import create_drake_scene
from drake_ompl.collision_checking import CollisionChecker

###############################
# Benchmarking Configurations #
###############################

PLANNERS = [
    "RRT",
    "EST",
    "RRTConnect",
    "RRTstar",
    "PRMstar",
    "FMTstar",
    "InformedRRTstar",
    "BITstar",
    # "GreedyRRTstar",
]

RUNTIME_LIMIT = 10
MEMORY_LIMIT = 4096
RUN_COUNT = 10

#################################
# Start and Goal configurations #
#################################

# fmt: off
START_Q = np.array([-1.02, 1.76,  0.86, -0.07, 0.00, 3.75, 2.89, 0.0, 0.0,
                    1.02, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.0, 0.0])

GOAL_Q = np.array([ 0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00, 0.0, 0.0,
                    -0.20, 1.70,  1.45, -0.87, 0.00, 1.40, 1.30, 0.0, 0.0])
# fmt: on

#############################################################################

# DO NOT CHANGE ANYTHING BELOW


def allocatePlanner(si, plannerType):
    if plannerType.lower() == "rrt":
        return og.RRT(si)
    elif plannerType.lower() == "est":
        return og.EST(si)
    elif plannerType.lower() == "rrtconnect":
        return og.RRTConnect(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "greedyrrtstar":
        return og.GreedyRRTstar(si)
    else:
        ou.OMPL_ERROR("Planner-type is not implemented in allocation function.")


def create_planners(ss, planner_names):
    planners = []
    si = ss.getSpaceInformation()
    for name in planner_names:
        planners.append(allocatePlanner(si, name))
    return planners


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, n_joints, space_information, collision_checker):
        super().__init__(space_information)
        self.collision_checker = collision_checker
        self.n_joints = n_joints

    def isValid(self, state):
        q = np.array([float(state[i]) for i in range(self.n_joints)])
        return self.collision_checker.is_valid(q)


def benchmark_planners(
    ss,
    planners,
    runtime_limit=10,
    memory_limit=4096,
    run_count=5,
    log_file="",
):
    """
    Runs the benchmark on the given planners and saves the results.
    """
    request = ot.Benchmark.Request(runtime_limit, memory_limit, run_count)
    benchmark = ot.Benchmark(ss, "Benchmarking OMPL Planners")
    for planner in planners:
        benchmark.addPlanner(planner)
    benchmark.benchmark(request)
    benchmark.saveResultsToFile(log_file)
    print(f"Benchmark results saved to {log_file}")


def main():

    ## Drake setup
    (
        plant,
        scene_graph,
        diagram,
        diagram_context,
        plant_context,
        scene_graph_context,
        teleop,
        meshcat,
    ) = create_drake_scene(1e-4, True)

    # setup collision checker
    collision_checker = CollisionChecker(
        plant, plant_context, scene_graph, scene_graph_context
    )

    plant.SetPositions(plant_context, START_Q)
    teleop.SetPositions(START_Q)
    diagram.ForcedPublish(diagram_context)

    ## OMPL setup

    num_joints = plant.num_positions()
    space = ob.RealVectorStateSpace(num_joints)

    ## set joint limits
    bounds = ob.RealVectorBounds(num_joints)
    lower_bounds = plant.GetPositionLowerLimits()
    upper_bounds = plant.GetPositionUpperLimits()
    for i in range(num_joints):
        bounds.setLow(i, lower_bounds[i])
        bounds.setHigh(i, upper_bounds[i])
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)

    validity_checker = ValidityChecker(
        num_joints, ss.getSpaceInformation(), collision_checker
    )
    ss.setStateValidityChecker(validity_checker)
    ss.getSpaceInformation().setStateValidityCheckingResolution(0.001)

    start_state = ob.State(space)
    goal_state = ob.State(space)
    for i in range(num_joints):
        start_state[i] = START_Q[i]
        goal_state[i] = GOAL_Q[i]
    ss.setStartAndGoalStates(start_state, goal_state)

    # Run benchmarking

    # this file directory
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # benchmark directory in top-level directory of project
    benchmarks_dir = os.path.join(os.path.dirname(current_dir), "benchmarks")
    os.makedirs(benchmarks_dir, exist_ok=True)

    # current benchmark directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamp_dir = os.path.join(benchmarks_dir, timestamp)
    os.makedirs(timestamp_dir, exist_ok=True)

    # paths to benchmark log file and database
    log_file_path = os.path.join(timestamp_dir, "benchmark.log")
    db_file_path = os.path.join(timestamp_dir, "benchmark.db")

    planners = create_planners(ss, PLANNERS)
    benchmark_planners(
        ss,
        planners,
        runtime_limit=RUNTIME_LIMIT,
        memory_limit=MEMORY_LIMIT,
        run_count=RUN_COUNT,
        log_file=log_file_path,
    )

    # convert the log file into db

    # ompl_benchmark_statistics.py script
    statistics_script = os.path.join(
        current_dir, "drake_ompl", "ompl_benchmark_statistics.py"
    )

    # python ompl_benchmark_statistics.py benchmark_ompl.log -d benchmark_ompl.db
    command = ["python", statistics_script, log_file_path, "-d", db_file_path]
    print("Executing:", " ".join(command))
    subprocess.run(command, capture_output=True, text=True)


if __name__ == "__main__":
    main()
