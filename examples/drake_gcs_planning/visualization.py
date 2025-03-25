# implementation based on:
# https://github.com/mpetersen94/gcs/blob/main/reproduction/bimanual/helpers.py
# https://github.com/mpetersen94/gcs/blob/main/reproduction/prm_comparison/helpers.py

import numpy as np

from pydrake.all import MultibodyPlant
from pydrake.perception import PointCloud
from pydrake.systems.analysis import Simulator
from pydrake.systems.rendering import MultibodyPositionToGeometryPose
from pydrake.trajectories import PiecewisePolynomial
from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import Meshcat, MeshcatVisualizer, Rgba, SceneGraph
from pydrake.systems.framework import LeafSystem

class VectorTrajectorySource(LeafSystem):
    def __init__(self, trajectories):
        LeafSystem.__init__(self)
        self.trajectories = trajectories
        self.start_time = [0]
        for traj in trajectories:
            self.start_time.append(self.start_time[-1] + traj.end_time())
        self.start_time = np.array(self.start_time)
        self.port = self.DeclareVectorOutputPort("traj_eval", 18, self.DoVecTrajEval, {self.time_ticket()})

    def DoVecTrajEval(self, context, output):
        t = context.get_time()
        traj_index = np.argmax(self.start_time > t) - 1

        q = self.trajectories[traj_index].value(t - self.start_time[traj_index])
        output.set_value(q)

def make_traj(path: np.array,
              speed: float = 2) -> PiecewisePolynomial.FirstOrderHold:
    """Returns a trajectory for the given path with constant speed.

    Args:
        path: The path to be turned into a trajectory.
        speed: The speed of the trajectory.
    Returns:
        A trajectory for the given path with constant speed.
    """
    t_breaks = [0]
    distance_between_knots = np.sqrt(
        np.sum(np.square(path.T[1:, :] - path.T[:-1, :]), axis=1))
    for segment_duration in distance_between_knots / speed:
        # Add a small number to ensure all times are increasing.
        t_breaks += [segment_duration + 1e-6 + t_breaks[-1]]
    return PiecewisePolynomial.FirstOrderHold(t_breaks, path)

def visualize_trajectory(traj, draw_ee_path=False, rgb_color = (0, 0, 0, 1)):
    meshcat = Meshcat()
    builder = DiagramBuilder()
    scene_graph = builder.AddSystem(SceneGraph())
    plant = MultibodyPlant(time_step=0.0)
    plant.RegisterAsSourceForSceneGraph(scene_graph)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("./examples/models/package.xml")
    directives = LoadModelDirectives("./examples/models/two_panda_arms_scene.yaml")
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    arm_1, arm_2 = models[:2]

    to_pose = builder.AddSystem(MultibodyPositionToGeometryPose(plant))
    builder.Connect(to_pose.get_output_port(), scene_graph.get_source_pose_port(plant.get_source_id()))
    traj_system = builder.AddSystem(VectorTrajectorySource(traj))
    end_time = np.sum([t.end_time() for t in traj])

    builder.Connect(traj_system.get_output_port(), to_pose.get_input_port())
    meshcat_viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Delete()
    vis_diagram = builder.Build()
    simulator = Simulator(vis_diagram)

    if draw_ee_path:
        plant_context = plant.CreateDefaultContext()
        arm_1_X = []
        arm_2_X = []
        for t in traj:
            q_waypoints = t.vector_values(np.linspace(t.start_time(), t.end_time(), 1000))
            for ii in range(q_waypoints.shape[1]):
                plant.SetPositions(plant_context, q_waypoints[:, ii])
                arm_1_X.append(plant.EvalBodyPoseInWorld(
                    plant_context, plant.GetBodyByName("panda_hand", arm_1.model_instance)))
                arm_2_X.append(plant.EvalBodyPoseInWorld(
                    plant_context, plant.GetBodyByName("panda_hand", arm_2.model_instance)))

        arm1_pointcloud = PointCloud(len(arm_1_X))
        arm1_pointcloud.mutable_xyzs()[:] = np.array(
            list(map(lambda X: X.translation(), arm_1_X))).T[:]
        meshcat.SetObject("paths/left_arm", arm1_pointcloud, 0.015, rgba=Rgba(*rgb_color))
        arm2_pointcloud = PointCloud(len(arm_2_X))
        arm2_pointcloud.mutable_xyzs()[:] = np.array(
            list(map(lambda X: X.translation(), arm_2_X))).T[:]
        meshcat.SetObject("paths/right_arm", arm2_pointcloud, 0.015, rgba=Rgba(*rgb_color))

    meshcat_viz.StartRecording()
    simulator.AdvanceTo(end_time)
    meshcat_viz.PublishRecording()
