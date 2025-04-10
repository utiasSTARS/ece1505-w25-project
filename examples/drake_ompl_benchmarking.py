import numpy as np
import os
import pickle
import time
from datetime import datetime

from ompl import base as ob
from ompl import geometric as og
from ompl import tools as ot
from ompl import util as ou

from drake_ompl.setup_scene import create_drake_scene
from drake_ompl.collision_checking import CollisionChecker

# todo: move these configs out from gcs folder
from drake_gcs_planning.convex_sets_config import get_configurations

# this file directory
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

###############################
# Benchmarking Configurations #
###############################

PLANNERS = [
    "PRM",  # only PRM is supported at the moment
]

# this serves as a timeout value
# planning will stop as soon as it found an initial solution
RUNTIME_LIMIT = 3

RUN_COUNT = 1
SHORTCUTTING = True
PRECOMPUTE = False
PRECOMPUTATION_TIME = 10

# this is used when PRECOMPUTE is set to False

STORAGE_PATHS = {
    "PRM": os.path.join(
        CURRENT_DIR, "../data/PRM-600s.graph"
    ),  # 600s precomputed roadmap
}

#################################
# Start and Goal configurations #
#################################

# fmt: off
START_Q = np.array([-1.09, 1.76,  0.86, -0.07, 0.00, 3.75, 2.89,  0.02, 0.02,
                     1.09, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.02, 0.02])

GOAL_Q = np.array([ 0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00, 0.04, 0.04,
                   -0.20, 1.70, 1.45, -0.87, 0.00, 1.40, 1.30, 0.04, 0.04])
# fmt: on

#############################################################################

# DO NOT CHANGE ANYTHING BELOW


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, n_joints, space_information, collision_checker):
        super().__init__(space_information)
        self.collision_checker = collision_checker
        self.n_joints = n_joints

    def isValid(self, state):
        q = np.array([float(state[i]) for i in range(self.n_joints)])
        return self.collision_checker.is_valid(q)


def allocate_planner(si, planner_id):
    if planner_id.lower() == "prm":
        return og.PRM(si)
    else:
        ou.OMPL_ERROR("Planner ID is not implemented in allocate_planner function.")


def allocate_persistent_planner(data, planner_id):
    if planner_id.lower() == "prm":
        return og.PRM(data)
    elif planner_id.lower() == "lazyprm":
        return og.LazyPRM(data)
    # TODO: SPARS and SPARS2 work differently for reusing data
    else:
        ou.OMPL_ERROR(
            "Planner ID is not implemented in allocate_persistent_planner function."
        )


def planner_allocator_impl(si, planner_id, storage_path=None, data=None):
    planner = None

    # we load the precomputed data into planner from storage path
    if storage_path is not None:
        data = ob.PlannerData(si)
        storage = ob.PlannerDataStorage()
        storage.load(storage_path, data)
        print(
            f"Loading planner data. NumEdges: {data.numEdges()}, NumVertices: {data.numVertices()}"
        )
        planner = allocate_persistent_planner(data, planner_id)
    # if data is already given, we load that too
    elif data is not None:
        print(
            f"Loading planner data. NumEdges: {data.numEdges()}, NumVertices: {data.numVertices()}"
        )
        planner = allocate_persistent_planner(data, planner_id)
    else:
        planner = allocate_planner(si, planner_id)

    return planner


def generate_fixed_configurations(space):
    for q in get_configurations().values():
        # remember to use "space.allocState()" instead of "state = ob.State(space)"
        # this will create new state object, the other will one reuse the same object pointer so it wont work
        state = space.allocState()
        for i in range(len(q)):
            state[i] = q[i]
        yield state


def benchmark_planners(
    ss,
    start_state,
    goal_state,
    planners,
    storage_paths,
    runtime_limit=10,
    run_count=5,
    data_file_path="",
):
    """
    Runs the benchmark on the given planners and saves the results.
    """
    execution_times = []
    solution_costs = []
    solutions = []

    for planner_id in planners:
        run_id = 0
        while run_id < run_count:

            planner = planner_allocator_impl(
                ss.getSpaceInformation(),
                planner_id,
                storage_paths.get(planner_id, None),
            )

            ## setup planner
            pdef = ob.ProblemDefinition(ss.getSpaceInformation())

            ## path length optimizing objective
            if planner_id != "PRM":  ## dont set this for PRM
                pdef.setOptimizationObjective(
                    ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
                )

            pdef.setStartAndGoalStates(start_state, goal_state)
            planner.setProblemDefinition(pdef)
            planner.setup()

            ## termination condition
            ## 1) this condition will terminate the planning as soon as planner founds the exact solution
            ptc1 = ob.exactSolnPlannerTerminationCondition(pdef)
            ## 2) this condition will terminate the planning upon timeout
            ptc2 = ob.timedPlannerTerminationCondition(runtime_limit)
            ## either one of the conditions needs to satisfy
            ptc = ob.plannerOrTerminationCondition(ptc1, ptc2)

            ## solve the planning problem
            start_time = time.time()
            solved = planner.solve(ptc)
            execution_times.append(time.time() - start_time)

            solution = []
            if solved:
                path = pdef.getSolutionPath()

                # perform shortcutting based on RRT-Rope approach
                if SHORTCUTTING:
                    path_simplifier = og.PathSimplifier(ss.getSpaceInformation())
                    start_time = time.time()
                    path_simplifier.ropeShortcutPath(path)
                    shortcutting_time = time.time() - start_time
                    execution_times[-1] += shortcutting_time

                ndim = ss.getSpaceInformation().getStateDimension()
                for i in range(path.getStateCount()):
                    state = path.getState(i)
                    configuration = [state[j] for j in range(ndim)]
                    solution.append(configuration)

                # compute solution cost
                total_length = 0
                for i in range(len(solution) - 1):
                    total_length += np.linalg.norm(
                        np.array(solution[i]) - np.array(solution[i + 1])
                    )
                solution_costs.append(total_length)
                solutions.append(solution)
            else:
                solution_costs.append(float("inf"))
                solutions.append(None)
            run_id += 1

    saved_data = {
        "solution": solutions[0],
        "solution_length": solution_costs,
        "time": execution_times,
    }
    with open(data_file_path, "wb") as f:
        pickle.dump(saved_data, f)
    print(f"Benchmark results saved to {data_file_path}")


def main():

    # Run benchmarking

    # benchmark directory in top-level directory of project
    benchmarks_dir = os.path.join(os.path.dirname(CURRENT_DIR), "benchmarks")
    os.makedirs(benchmarks_dir, exist_ok=True)

    # current benchmark directory
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamp_dir = os.path.join(benchmarks_dir, timestamp)
    os.makedirs(timestamp_dir, exist_ok=True)

    # paths to benchmark database
    data_file_path = os.path.join(timestamp_dir, "benchmark.pkl")

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

    pdef = ob.ProblemDefinition(ss.getSpaceInformation())
    # careful with setting pdef here (PRM wont work with this especially for ptc)
    pdef.setOptimizationObjective(
        ob.PathLengthOptimizationObjective(ss.getSpaceInformation())
    )
    pdef.setStartAndGoalStates(start_state, goal_state)

    # Perform pre-computation of roadmaps if specified
    storage_paths = {}
    if PRECOMPUTE:
        # only consider roadmap based ones
        planners_to_precompute = []
        for planner_id in PLANNERS:
            # for roadmap based ones, we also include pre-specified
            # robot configurations used during GCS
            data = ob.PlannerData(ss.getSpaceInformation())
            for q in generate_fixed_configurations(ss.getSpaceInformation()):
                v = ob.PlannerDataVertex(q)
                data.addVertex(v)
            planners_to_precompute.append(
                (
                    planner_id,
                    planner_allocator_impl(
                        ss.getSpaceInformation(),
                        planner_id,
                        storage_path=None,
                        data=data,
                    ),
                )
            )

        # execute precomputation of roadmaps
        print(
            "Performing precomputation of roadmaps. This will take some time to complete..."
        )
        for planner_id, planner in planners_to_precompute:

            ## setup planner
            planner.setProblemDefinition(pdef)
            planner.setup()

            ## run the planner
            ## here we directly provide time to the solve() call without using any ptc
            ## this will run this planner until timeout
            planner.solve(PRECOMPUTATION_TIME)

            # store the roadmap
            data = ob.PlannerData(ss.getSpaceInformation())
            planner.getPlannerData(data)
            print(
                f"Storing precomputed planner data. NumEdges: {data.numEdges()}, NumVertices: {data.numVertices()}"
            )
            storage_path = os.path.join(timestamp_dir, f"{planner_id}.graph")
            storage = ob.PlannerDataStorage()
            storage.store(data, storage_path)
            storage_paths[planner_id] = storage_path
            print(f"Precomputed roadmap graph saved at {storage_path}")
    else:
        storage_paths = STORAGE_PATHS

    ## benchmarking stuffs below

    benchmark_planners(
        ss,
        start_state,
        goal_state,
        PLANNERS,
        storage_paths=storage_paths,
        runtime_limit=RUNTIME_LIMIT,
        run_count=RUN_COUNT,
        data_file_path=data_file_path,
    )


if __name__ == "__main__":
    main()
