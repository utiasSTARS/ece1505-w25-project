import numpy as np
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

import pickle as pkl
from drake_gcs_planning.visualization import make_traj, visualize_trajectory
import time

TRAJ_TYPE = "LinearGCS" #"PRM", "LinearGCS", "BezierGCS"

if __name__ == '__main__':
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("./examples/models/package.xml")
    directives = LoadModelDirectives("./examples/models/two_panda_arms_scene.yaml")
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    if TRAJ_TYPE == "PRM":
        pkl_file_name = "./data/benchmark_prm.pkl"
        with open(pkl_file_name, "rb") as f:
            pkl_dict = pkl.load(f)
        # print(pkl_dict.keys())
        traj = np.array(pkl_dict["solution"]).T
        traj = make_traj(traj)
    elif TRAJ_TYPE == "LinearGCS":
        pkl_file_name = "./data/benchmark_gcs_18.pkl"
        with open(pkl_file_name, "rb") as f:
            pkl_dict = pkl.load(f)
        rounded_cost = np.array(pkl_dict["rounded_cost"])
        min_cost_idx = np.argmin(rounded_cost)
        # print(rounded_cost[min_cost_idx], min_cost_idx)
        traj = np.array(pkl_dict["traj"][min_cost_idx])
        traj = make_traj(traj)
    elif TRAJ_TYPE == "BezierGCS":
        pass


    visualize_trajectory([traj], True)
    time.sleep(100)
