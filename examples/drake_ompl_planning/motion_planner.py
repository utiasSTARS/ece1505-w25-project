import numpy as np

from ompl import base as ob
from ompl import geometric as og

from pydrake.all import MultibodyPlant

from collision_checking import CollisionChecker


class OMPLPlanner:
    def __init__(self, plant: MultibodyPlant, collision_checker: CollisionChecker):
        self.plant = plant
        self.collision_checker = collision_checker
        self.num_joints = plant.num_positions()
        self.space = ob.RealVectorStateSpace(self.num_joints)
        self.si = ob.SpaceInformation(self.space)

        ## set joint limits
        bounds = ob.RealVectorBounds(self.num_joints)
        lower_bounds = plant.GetPositionLowerLimits()
        upper_bounds = plant.GetPositionUpperLimits()
        # print(f"lower bounds: {lower_bounds}")
        # print(f"upper bounds: {upper_bounds}")
        for i in range(self.num_joints):
            bounds.setLow(i, lower_bounds[i])
            bounds.setHigh(i, upper_bounds[i])
        self.space.setBounds(bounds)

        self.si = ob.SpaceInformation(self.space)

        class ValidityChecker(ob.StateValidityChecker):
            def __init__(self, n_joints, space_information, collision_checker):
                super().__init__(space_information)
                self.collision_checker = collision_checker
                self.n_joints = n_joints

            def isValid(self, state):
                q = np.array([float(state[i]) for i in range(self.n_joints)])
                return self.collision_checker.is_valid(q)

        self.validity_checker = ValidityChecker(
            self.num_joints, self.si, self.collision_checker
        )
        self.si.setStateValidityChecker(self.validity_checker)
        self.si.setStateValidityCheckingResolution(0.01)
        self.si.setup()

    def setup(
        self,
        start_q: np.ndarray,
        goal_q: np.ndarray,
        planner_type=og.RRTConnect,
    ):
        ## create ProblemDefinition.
        self.pdef = ob.ProblemDefinition(self.si)

        # Set start and goal states.
        start_state = ob.State(self.space)
        goal_state = ob.State(self.space)
        for i in range(self.num_joints):
            start_state[i] = start_q[i]
            goal_state[i] = goal_q[i]
        self.pdef.setStartAndGoalStates(start_state, goal_state)

        ## create planner
        self.planner = planner_type(self.si)
        self.planner.setProblemDefinition(self.pdef)
        self.planner.setup()

    def plan(self, planning_time: float = 10.0) -> list | None:

        solved = self.planner.solve(planning_time)

        if solved:
            print("Solution found!")
            path = self.pdef.getSolutionPath()
            path.interpolate()
            solution = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                configuration = [state[j] for j in range(self.num_joints)]
                solution.append(configuration)
            return solution
        else:
            print("No solution found.")
            return None
