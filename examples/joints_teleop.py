# This code was written based on the following examples:
# https://deepnote.com/workspace/Manipulation-ac8201a1-470a-4c77-afd0-2cc45bc229ff/project/8f86172b-b597-4ceb-9bad-92d11ac7a6cc/notebook/simulation-1ba6290623e34dbbb9d822a2180187c1
# https://deepnote.com/workspace/Manipulation-ac8201a1-470a-4c77-afd0-2cc45bc229ff/project/8f86172b-b597-4ceb-9bad-92d11ac7a6cc/notebook/03_direct_joint_control-7f1093ba508342628a078d0d8804ec8b
# https://github.com/achuwilson/pydrake_iiwa/blob/main/example_joint_slider.py

import numpy as np
from fontTools.misc.cython import returns

from pydrake.geometry import Meshcat, MeshcatVisualizer

from pydrake.multibody.parsing import Parser, LoadModelDirectives, ProcessModelDirectives
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.meshcat import JointSliders

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import FirstOrderLowPassFilter, Multiplexer, ConstantVectorSource
from pydrake.systems.controllers import InverseDynamicsController

from absl import flags, app

FLAGS = flags.FLAGS
flags.DEFINE_string("initial_conf", "start", "Can use 'zeros', 'start', 'goal'")

def main(_):
    meshcat = Meshcat()
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
    parser = Parser(plant)
    parser.package_map().AddPackageXml("./examples/models/package.xml")
    directives = LoadModelDirectives("./examples/models/two_panda_arms_scene.yaml")
    models = ProcessModelDirectives(directives, plant, parser)
    plant.Finalize()

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    #joints teleoperation
    num_joints = plant.num_positions()
    teleop = builder.AddSystem(JointSliders(meshcat, plant))
    filter = builder.AddSystem(FirstOrderLowPassFilter(time_constant=1.00, size=num_joints))
    #joints controller
    kp = [100] * plant.num_positions()
    ki = [1] * plant.num_positions()
    kd = [20] * plant.num_positions()
    frankas_controller = builder.AddSystem(InverseDynamicsController(plant, kp, ki, kd, False))
    frankas_controller.set_name("frankas_controller")

    #estimated state
    builder.Connect(
        plant.get_state_output_port(),
        frankas_controller.get_input_port_estimated_state(),
    )

    #filtered joint slider values -> the desired state in terms of positions
    builder.Connect(teleop.get_output_port(),
                    filter.get_input_port())

    #add zero joint velocities to the desired state
    mux = builder.AddSystem(Multiplexer(input_sizes=[num_joints, num_joints]))
    zero_vector = builder.AddSystem(ConstantVectorSource([0] * num_joints))
    builder.Connect(filter.get_output_port(),
                    mux.get_input_port(0))
    builder.Connect(zero_vector.get_output_port(),
                    mux.get_input_port(1))

    #send the desired state to the controller
    builder.Connect(mux.get_output_port(),
                    frankas_controller.get_input_port_desired_state())

    #control
    builder.Connect(
        frankas_controller.get_output_port_control(),
        plant.get_actuation_input_port()
    )

    diagram = builder.Build()
    diagram.set_name("with inverse dynamics controller")

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)

    if FLAGS.initial_conf == "zeros":
        q0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0, 0, 0, 0])
    elif FLAGS.initial_conf == "start":
        # robot arms are between the first and second rows of obstacles
        # (if we count starting from the closest ones to the robot)
        q0 = np.array([-1.02, 1.76,  0.86, -0.07, 0.00, 3.75, 2.9,  0.00, 0.00,
                        1.02, 1.76, -0.86, -0.07, 0.00, 3.75, 2.05, 0.00, 0.00])
    elif FLAGS.initial_conf == "goal":
        # robot arms are between the second and third rows of obstacles
        # (if we count starting from the closest ones to the robot)
        q0 = np.array([ 0.20, 1.70, -1.45, -0.87, 0.00, 1.40, 0.00,  0.00, 0.00,
                       -0.20, 1.70,  1.45, -0.87, 0.00, 1.40, 1.30,  0.00, 0.00])
    else:
        print("Error: Initial conf not recognized")
        return

    plant.SetPositions(plant_context, q0)
    teleop.SetPositions(q0)

    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(np.inf)

if __name__ == "__main__":
    app.run(main)