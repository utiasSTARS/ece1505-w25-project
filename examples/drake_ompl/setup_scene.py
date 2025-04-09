import os

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ConstantVectorSource,
    DiagramBuilder,
    FirstOrderLowPassFilter,
    InverseDynamicsController,
    JointSliders,
    MeshcatVisualizer,
    Multiplexer,
    StartMeshcat,
    Parser,
    LoadModelDirectives,
    ProcessModelDirectives,
)


def create_drake_scene(time_step, visualize=True):
    """Creates the MultibodyPlant and SceneGraph for the robot and environment."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    parser = Parser(plant)

    ## use absolute paths for xml and yaml files
    current_dir = os.path.dirname(os.path.abspath(__file__))
    package_xml_path = os.path.join(current_dir, "..", "models", "package.xml")
    scene_file = os.path.join(current_dir, "..", "models", "two_panda_arms_scene.yaml")

    parser.package_map().AddPackageXml(package_xml_path)
    directives = LoadModelDirectives(scene_file)
    models = ProcessModelDirectives(directives, plant, parser)

    plant.Finalize()

    if visualize:
        meshcat = StartMeshcat()

        MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # control

    num_joints = plant.num_positions()
    teleop = builder.AddSystem(JointSliders(meshcat, plant))
    filter = builder.AddSystem(
        FirstOrderLowPassFilter(time_constant=1.00, size=num_joints)
    )

    ## joints controller
    kp = [100] * num_joints
    ki = [1] * num_joints
    kd = [20] * num_joints
    frankas_controller = builder.AddSystem(
        InverseDynamicsController(plant, kp, ki, kd, False)
    )
    frankas_controller.set_name("frankas_controller")

    ## estimated state
    builder.Connect(
        plant.get_state_output_port(),
        frankas_controller.get_input_port_estimated_state(),
    )

    # filtered joint slider values -> the desired state in terms of positions
    builder.Connect(teleop.get_output_port(), filter.get_input_port())

    ## add zero joint velocities to the desired state
    mux = builder.AddSystem(Multiplexer(input_sizes=[num_joints, num_joints]))
    zero_vector = builder.AddSystem(ConstantVectorSource([0] * num_joints))
    builder.Connect(filter.get_output_port(), mux.get_input_port(0))
    builder.Connect(zero_vector.get_output_port(), mux.get_input_port(1))

    ## send the desired state to the controller
    builder.Connect(
        mux.get_output_port(), frankas_controller.get_input_port_desired_state()
    )

    ## control
    builder.Connect(
        frankas_controller.get_output_port_control(), plant.get_actuation_input_port()
    )

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    scene_graph_context = scene_graph.GetMyMutableContextFromRoot(diagram_context)

    return (
        plant,
        scene_graph,
        diagram,
        diagram_context,
        plant_context,
        scene_graph_context,
        teleop,
        meshcat if visualize else None,
    )
