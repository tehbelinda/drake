import argparse
import math
import numpy as np

from pydrake.geometry import SceneGraph
from pydrake.examples.acrobot import (AcrobotGeometry, AcrobotInput,
    AcrobotPlant, AcrobotState)
from pydrake.systems.analysis import Simulator
from pydrake.systems.controllers import LinearQuadraticRegulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.planar_scenegraph_visualizer import (
    PlanarSceneGraphVisualizer)
from pydrake.systems.primitives import Saturation, WrapToSystem


def UprightState():
    state = AcrobotState()
    state.set_theta1(math.pi)
    state.set_theta2(0.)
    state.set_theta1dot(0.)
    state.set_theta2dot(0.)
    return state


def BalancingLQR():
    # Design an LQR controller for stabilizing the Acrobot around the upright.
    # Returns a (static) AffineSystem that implements the controller (in
    # the original AcrobotState coordinates).

    acrobot = AcrobotPlant()
    context = acrobot.CreateDefaultContext()

    input = AcrobotInput()
    input.set_tau(0.)
    context.FixInputPort(0, input)

    context.get_mutable_continuous_state_vector()\
        .SetFromVector(UprightState().CopyToVector())

    Q = np.diag((10., 10., 1., 1.))
    R = [1]

    return LinearQuadraticRegulator(acrobot, context, Q, R)


if __name__ == "__main__":
    builder = DiagramBuilder()

    acrobot = builder.AddSystem(AcrobotPlant())
    saturation = builder.AddSystem(Saturation(min_value=[-10], max_value=[10]))
    builder.Connect(saturation.get_output_port(0), acrobot.get_input_port(0))
    wrapangles = WrapToSystem(4)
    wrapangles.set_interval(0, 0, 2. * math.pi)
    wrapangles.set_interval(1, -math.pi, math.pi)
    wrapto = builder.AddSystem(wrapangles)
    builder.Connect(acrobot.get_output_port(0), wrapto.get_input_port(0))
    controller = builder.AddSystem(BalancingLQR())
    builder.Connect(wrapto.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), saturation.get_input_port(0))

    scene_graph = builder.AddSystem(SceneGraph())
    AcrobotGeometry.AddToBuilder(builder, acrobot.get_output_port(0),
                                 scene_graph)
    visualizer = builder.AddSystem(
        PlanarSceneGraphVisualizer(scene_graph, xlim=[-4., 4.], ylim=[-4., 4.]))
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    visualizer.get_input_port(0))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    context = simulator.get_mutable_context()

    parser = argparse.ArgumentParser()
    parser.add_argument("-N",
                        "--trials",
                        type=int,
                        help="Number of trials to run.",
                        default=5)
    parser.add_argument("-T",
                        "--duration",
                        type=float,
                        help="Duration to run each sim.",
                        default=4.0)
    args = parser.parse_args()

    for i in range(args.trials):
        context.SetTime(0.)
        context.SetContinuousState(UprightState().CopyToVector() +
                                   0.05 * np.random.randn(4,))
        simulator.Initialize()
        simulator.AdvanceTo(args.duration)
