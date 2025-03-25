from pydrake.all import MultibodyPlant
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import MosekSolver

from gcs.bezier import BezierGCS
from gcs.linear import LinearGCS
from gcs.rounding import randomForwardPathSearch

from convex_sets_config import *
import pickle as pkl
import configparser # https://www.geeksforgeeks.org/how-to-write-a-configuration-file-in-python/
CONFIG_PATH = "./examples/drake_gcs_planning/config.ini"

from visualization import make_traj, visualize_trajectory
import time

class GCSPlanner:
    def __init__(self, plant: MultibodyPlant):
        self.plant = plant
        self.cfg = self._read_config()
        self.regions = self._get_iris_regions(self.cfg["full_graph"])


    def setup(
        self,
        start_q: np.ndarray,
        goal_q: np.ndarray
    ):
        if self.cfg["planner_type"] == "LinearGCS":
            self.gcs = self._setup_linear()
        elif self.cfg["planner_type"] == "BezierGCS":
            self.gcs = self._setup_bezier()

        self.gcs.addSourceTarget(start_q, goal_q)
        self.gcs.setPaperSolverOptions()
        self.gcs.setSolver(MosekSolver())
        if self.cfg["planner_type"] == "LinearGCS":
            self.gcs.options.solver_options.SetOption(MosekSolver.id(),
                                                     'MSK_DPAR_INTPNT_TOL_PFEAS',
                                                      self.cfg["solver_tolerance"])
            self.gcs.options.solver_options.SetOption(MosekSolver.id(),
                                                     'MSK_DPAR_INTPNT_TOL_DFEAS',
                                                      self.cfg["solver_tolerance"])
            self.gcs.options.solver_options.SetOption(MosekSolver.id(),
                                                     'MSK_DPAR_INTPNT_TOL_REL_GAP',
                                                      self.cfg["solver_tolerance"])
            self.gcs.options.solver_options.SetOption(MosekSolver.id(),
                                                     'MSK_DPAR_INTPNT_TOL_INFEAS',
                                                      self.cfg["solver_tolerance"])

        # other rounding strategies:
        # https://github.com/mpetersen94/gcs/blob/bdfe94f9233eeb5a425ebd3194a5915fa0570c39/gcs/rounding.py#L71
        self.gcs.setRoundingStrategy(randomForwardPathSearch, max_paths=10, max_trials=100, seed=0)


    def _read_config(self):
        config = configparser.ConfigParser()
        config.read(CONFIG_PATH)

        cfg = {
            "planner_type": config.get('General', 'planner_type'),
            "full_graph": config.getboolean('General', 'full_graph'),
        }

        if cfg["planner_type"] == "LinearGCS":
            path_weights = config.get('LinearGCS', 'path_weights')
            if path_weights == "None":
                cfg["path_weights"] = None
            else:
                cfg["path_weights"] = path_weights
            cfg["full_dim_overlap"] = config.getboolean('LinearGCS', 'full_dim_overlap')
            cfg["solver_tolerance"] = config.getfloat('LinearGCS', 'solver_tolerance')
        elif cfg["planner_type"] == "BezierGCS":
            cfg["order"] = config.getint('BezierGCS', 'order')
            cfg["continuity"] = config.getint('BezierGCS', 'continuity')
            cfg["hdot_min"] = config.getfloat('BezierGCS', 'hdot_min')
            cfg["full_dim_overlap"] = config.getboolean('BezierGCS', 'full_dim_overlap')
            cfg["velocity_multiplier"] = config.getfloat('BezierGCS', 'velocity_multiplier')
            cfg["time_cost"] = config.getfloat('BezierGCS', 'time_cost')
            cfg["path_length_cost"] = config.getfloat('BezierGCS', 'path_length_cost')
            cfg["path_energy_cost"] = config.getfloat('BezierGCS', 'path_energy_cost')
        return cfg


    def _get_iris_regions(self, full_graph: bool):
        region_files = get_region_files()
        minimum_graph_nodes = get_minimum_graph_nodes()

        if not full_graph:
            region_files = {key: region_files[key] for key in minimum_graph_nodes}

        regions = {}
        for region_name, file in region_files.items():
            with open(file, "rb") as f:
                reg = pkl.load(f)
                regions[region_name] = reg
        return regions


    # https://github.com/mpetersen94/gcs/blob/main/reproduction/prm_comparison/prm_comparison.ipynb
    def _setup_linear(self):
        # Page 20:
        # The objective is to connect the start and the goal configurations with
        # a continuous (η := 0) trajectory of minimum Euclidean length (a := c := 0 and b := 1).
        # Velocity and time constraints are irrelevant given our objective.
        # It seems that LinearGCS corresponds to this implementation
        return LinearGCS(regions = self.regions,
                         path_weights = self.cfg["path_weights"],
                         full_dim_overlap = self.cfg["full_dim_overlap"])


    # https://github.com/mpetersen94/gcs/blob/main/reproduction/bimanual/helpers.py
    def _setup_bezier(self):
        gcs = BezierGCS(self.regions,
                        order = self.cfg["order"],
                        continuity = self.cfg["continuity"],
                        hdot_min = self.cfg["hdot_min"],
                        full_dim_overlap = self.cfg["full_dim_overlap"])
        gcs.addTimeCost(self.cfg["time_cost"])
        gcs.addPathLengthCost(self.cfg["path_length_cost"])
        gcs.addPathEnergyCost(self.cfg["path_energy_cost"])

        gcs.addDerivativeRegularization(1e-3, 1e-3, 2)
        gcs.addVelocityLimits(self.cfg["velocity_multiplier"] * self.plant.GetVelocityLowerLimits(),
                              self.cfg["velocity_multiplier"] * self.plant.GetVelocityUpperLimits())
        return gcs


    def plan(self, n_samples: int = 500, planning_time: float = 10.0) -> list | None:
        traj, results_dict = self.gcs.SolvePath(rounding=True,
                                                verbose=False,
                                                preprocessing=True)
        if traj is None:
            print("No solution found.")
            return None

        print("Solution found!")
        run_time = 0
        # run_time += results_dict["preprocessing_stats"]['linear_programs'] #-- was originally commented; TODO: why?
        run_time += results_dict["relaxation_solver_time"]
        run_time += results_dict["total_rounded_solver_time"] # for comparison with PRM they use "max_rounded_solver_time"
                                                              # because rounding can be parallelized; TODO
        print("\tRounded cost:", np.round(results_dict["rounded_cost"], 4),
              "\tRelaxed cost:", np.round(results_dict["relaxation_cost"], 4))
        print("\tCertified Optimality Gap:",
              (results_dict["rounded_cost"] - results_dict["relaxation_cost"])
              / results_dict["relaxation_cost"])
        self.gcs.ResetGraph()
        return traj

if __name__ == '__main__':
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("./examples/models/package.xml")
    directives = LoadModelDirectives("./examples/models/two_panda_arms_scene.yaml")
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()
    planner = GCSPlanner(plant)
    start_q = np.array([-1.09, 1.76,  0.86, -0.07, 0.00, 3.75, 2.89,  0.02, 0.02,
                         1.09, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.02, 0.02])
    goal_q = np.array([ 0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00, 0.04, 0.04,
                       -0.20, 1.70, 1.45, -0.87, 0.00, 1.40, 1.30, 0.04, 0.04])
    planner.setup(start_q, goal_q)
    traj = planner.plan()
    if planner.cfg["planner_type"] == "LinearGCS":
        traj = make_traj(traj)
    visualize_trajectory([traj], True)
    time.sleep(100)
