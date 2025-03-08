import numpy as np
import time

from ompl import geometric as og

from collision_checking import CollisionChecker
from motion_planner import OMPLPlanner
from setup_scene import create_drake_scene


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

    # if solution found
    if solution:
        print(f"Found solution of length: {len(solution)}")
        if visualize:
            for q in solution:
                plant.SetPositions(plant_context, q)
                diagram.ForcedPublish(diagram_context)
                time.sleep(0.1)  # playback speed

    if visualize:
        print("Press Enter to exit.")
        input()
    else:
        print("Finished without visualization.")


if __name__ == "__main__":
    main()
