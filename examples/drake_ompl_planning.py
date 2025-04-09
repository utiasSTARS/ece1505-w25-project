import numpy as np
import time

from ompl import geometric as og
from ompl import tools as ot

from pydrake.all import PiecewisePolynomial

from drake_ompl.collision_checking import CollisionChecker
from drake_ompl.motion_planner import OMPLPlanner
from drake_ompl.setup_scene import create_drake_scene


def main():
    time_step = 1e-4
    visualize = True  # set to False to disable visualization
    (
        plant,
        scene_graph,
        diagram,
        diagram_context,
        plant_context,
        scene_graph_context,
        teleop,
        meshcat,
    ) = create_drake_scene(time_step, visualize)

    # setup collision checker
    collision_checker = CollisionChecker(
        plant, plant_context, scene_graph, scene_graph_context
    )

    # fmt: off
    start_q = np.array([-1.02, 1.76,  0.86, -0.07, 0.00, 3.75, 2.89, 0.0, 0.0,
                        1.02, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.0, 0.0])

    goal_q = np.array([ 0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00, 0.0, 0.0,
                       -0.20, 1.70,  1.45, -0.87, 0.00, 1.40, 1.30, 0.0, 0.0])
    # fmt: on

    plant.SetPositions(plant_context, start_q)
    teleop.SetPositions(start_q)
    diagram.ForcedPublish(diagram_context)

    # create OMPL planner instance
    planner = OMPLPlanner(plant, collision_checker)

    # solve the planning problem
    planner.setup(start_q, goal_q, planner_type=og.RRTConnect)
    solution = planner.plan(planning_time=10.0)

    # compute solution cost
    total_length = 0
    for i in range(len(solution) - 1):
        total_length += np.sum((np.array(solution[i]) - np.array(solution[i + 1])) ** 2)
    print(f"Solution cost: {total_length}")

    # if solution found
    if solution:
        print(f"Found solution of length: {len(solution)}")
        if visualize:

            total_time = 10
            times = np.linspace(0, total_time, len(solution))
            trajectory = PiecewisePolynomial.FirstOrderHold(times, np.array(solution).T)

            start_time = time.time()
            while True:
                current_time = time.time() - start_time
                if current_time > total_time:
                    break

                desired_q = trajectory.value(current_time).flatten()
                plant.SetPositions(plant_context, desired_q)
                diagram.ForcedPublish(diagram_context)

                # simulation step
                time.sleep(0.01)

    if visualize:
        print("Press Enter to exit.")
        input()
    else:
        print("Finished without visualization.")


if __name__ == "__main__":
    main()
