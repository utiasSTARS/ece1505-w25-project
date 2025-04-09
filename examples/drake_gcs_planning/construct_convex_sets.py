from pydrake.geometry.optimization import IrisInConfigurationSpace, IrisOptions
from pydrake.geometry import Meshcat, QueryObject, Rgba, Role
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.solvers import MathematicalProgram, Solve

import time
import pydot
import pickle as pkl
import numpy as np

from convex_sets_config import *

from absl import flags, app
FLAGS = flags.FLAGS
flags.DEFINE_string("action", "gen_graph", "Can use 'gen_graph', 'gen_convex_sets', 'vis_convex_sets'")
flags.DEFINE_boolean("full_graph", False, "If True, all convex_sets will be used")
flags.DEFINE_string("region_name", "all",
                    "With this parameter you can choose which convex set to visualize; if 'all', all convex sets will be visualized")

class ConvexSetsConstructor:
    def __init__(self):
        self.meshcat = Meshcat()
        self.builder = DiagramBuilder()

        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=1e-4)
        parser = Parser(self.plant)
        parser.package_map().AddPackageXml("./examples/models/package.xml")
        directives = LoadModelDirectives("./examples/models/two_panda_arms_scene.yaml")
        models = ProcessModelDirectives(directives, self.plant, parser)
        self.plant.Finalize()

        self.diagram = self.builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.iris_options = IrisOptions()

        # from https://github.com/mpetersen94/gcs/blob/bdfe94f9233eeb5a425ebd3194a5915fa0570c39/reproduction/bimanual/bimanual_iiwa_example.ipynb#L103
        # takes more time to run so it was not used
        # self.iris_options.require_sample_point_is_contained = True
        # self.iris_options.iteration_limit = 5
        # self.iris_options.termination_threshold = -1
        # self.iris_options.relative_termination_threshold = 0.02
        #
        # self.iris_options.num_collision_infeasible_samples = 1

        # from https://github.com/vincekurtz/ltl_gcs/blob/main/examples/robot_arm.py
        # takes less time to run so it what was used
        self.iris_options.require_sample_point_is_contained = True
        self.iris_options.iteration_limit = 1
        self.iris_options.termination_threshold = 2e-2
        self.iris_options.relative_termination_threshold = 2e-2
        self.iris_options.num_collision_infeasible_samples = 1


    def calcRegion(self, conf_name, conf, context):
        start_time = time.time()
        diagram_context = self.diagram.CreateDefaultContext()
        plant_context = self.diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        self.plant.SetPositions(plant_context, conf)

        hpoly = IrisInConfigurationSpace(self.plant, plant_context, self.iris_options)
        print("Seed:", conf_name, "\tTime:", np.round((time.time() - start_time) / 60., 4),
              "minutes.\tFaces", len(hpoly.b()), flush=True)
        with open(f"./examples/drake_gcs_planning/iris_regions/iris_region_{conf_name}.pkl", "wb") as f:
            pkl.dump(hpoly, f)
        return hpoly


    def generateRegionsSequential(self, configurations):
        context_clones = [self.context.Clone() for _ in range(len(configurations))]
        context_idx = 0

        for conf_name, conf in configurations.items():
            print("Constructing region for", conf_name)
            self.calcRegion(conf_name, conf, context_clones[context_idx])
            context_idx += 1


    def DrawRobot(self, query_object: QueryObject, meshcat_prefix: str, draw_world: bool = True):
        rgba = Rgba(0.7, 0.7, 0.7, 0.3)
        role = Role.kProximity
        # This is a minimal replication of the work done in MeshcatVisualizer.
        inspector = query_object.inspector()
        for frame_id in inspector.GetAllFrameIds():
            if frame_id == inspector.world_frame_id():
                if not draw_world:
                    continue
                frame_path = meshcat_prefix
            else:
                frame_path = f"{meshcat_prefix}/{inspector.GetName(frame_id)}"
            frame_path.replace("::", "/")
            frame_has_any_geometry = False
            for geom_id in inspector.GetGeometries(frame_id, role):
                path = f"{frame_path}/{geom_id.get_value()}"
                path.replace("::", "/")
                self.meshcat.SetObject(path, inspector.GetShape(geom_id), rgba)
                self.meshcat.SetTransform(path, inspector.GetPoseInFrame(geom_id))
                frame_has_any_geometry = True

            if frame_has_any_geometry:
                X_WF = query_object.GetPoseInWorld(frame_id)
                self.meshcat.SetTransform(frame_path, X_WF)

    def visualizeRegion(self, iris_region, region_name, num_to_draw=30):
            """
            A simple hit-and-run-style idea for visualizing the IRIS regions:
            1. Start at the center. Pick a random direction and run to the boundary.
            2. Pick a new random direction; project it onto the current boundary, and run along it. Repeat
            """
            plant_context = self.plant.GetMyMutableContextFromRoot(self.context)
            scene_graph_context = self.scene_graph.GetMyContextFromRoot(self.context)

            q = iris_region.ChebyshevCenter()
            self.plant.SetPositions(plant_context, q)
            self.diagram.ForcedPublish(self.context)

            query = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
            self.DrawRobot(query, f"{region_name}/0", True)

            rng = np.random.default_rng()
            nq = self.plant.num_positions()
            prog = MathematicalProgram()
            qvar = prog.NewContinuousVariables(nq, "q")
            prog.AddLinearConstraint(iris_region.A(), 0 * iris_region.b() - np.inf, iris_region.b(), qvar)
            cost = prog.AddLinearCost(np.ones((nq, 1)), qvar)

            for i in range(1, num_to_draw):
                direction = rng.standard_normal(nq)
                cost.evaluator().UpdateCoefficients(direction)

                result = Solve(prog)
                assert result.is_success()

                q = result.GetSolution(qvar)
                self.plant.SetPositions(plant_context, q)
                query = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
                self.DrawRobot(query, f"{region_name}/{i}", False)

    def visualizeConnectivity(self, iris_regions):
        graph = pydot.Dot("IRIS region connectivity")
        keys = list(iris_regions.keys())
        for k in keys:
            graph.add_node(pydot.Node(k))
        for i in range(len(keys)):
            v1 = iris_regions[keys[i]]
            for j in range(i + 1, len(keys)):
                v2 = iris_regions[keys[j]]
                if v1.IntersectsWith(v2):
                    graph.add_edge(pydot.Edge(keys[i], keys[j], dir="both"))
        with open("./examples/drake_gcs_planning/graph.png", "wb") as f:
            f.write(graph.create_png())

def main(_):
    cs_constructor = ConvexSetsConstructor()

    region_files = get_region_files()
    minimum_graph_nodes = get_minimum_graph_nodes()

    if not FLAGS.full_graph:
        region_files = {key: region_files[key] for key in minimum_graph_nodes}

    if FLAGS.action == "gen_graph" or FLAGS.action == "vis_convex_sets":
        regions = {}
        for region_name, file in region_files.items():
            with open(file, "rb") as f:
                reg = pkl.load(f)
                regions[region_name] = reg

    if FLAGS.action == "gen_convex_sets":
        configurations = get_configurations()
        cs_constructor.generateRegionsSequential(configurations)
    elif FLAGS.action == "gen_graph":
        cs_constructor.visualizeConnectivity(regions)
    elif FLAGS.action == "vis_convex_sets":
        region_name = FLAGS.region_name
        if region_name == "all":
            for r_name in regions:
                cs_constructor.visualizeRegion(regions[r_name], r_name)
        else:
            cs_constructor.visualizeRegion(regions[region_name], region_name)
        time.sleep(100)


if __name__ == "__main__":
    app.run(main)
