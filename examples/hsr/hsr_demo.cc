#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace hsr {
namespace {

using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Translation3d;
using Eigen::VectorXd;

// Fixed path to double pendulum SDF model.
static const char* const kHsrUrdfPath =
    "drake/examples/hsr/models/hsrb4s.obj.urdf";

//
// Main function for demo.
//
int main(int argc, char* argv[]) {
  // Build a generic multibody plant.
  systems::DiagramBuilder<double> builder;
  auto pair = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(1.0E-3));
  MultibodyPlant<double>& plant = pair.plant;

  // Load and parse HSR URDF from file.
  const std::string urdf_path = FindResourceOrThrow(kHsrUrdfPath);
  multibody::Parser(&plant).AddModelFromFile(urdf_path);

  // Add model of the ground.
  const double static_friction = 1.0;
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(),
                               geometry::HalfSpace(), "GroundVisualGeometry",
                               green);
  // For a time-stepping model only static friction is used.
  const multibody::CoulombFriction<double> ground_friction(static_friction,
                                                           static_friction);
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  geometry::HalfSpace(),
                                  "GroundCollisionGeometry", ground_friction);

  plant.Finalize();

  const drake::multibody::Body<double>& body =
      plant.GetBodyByName("base_footprint");

  // Publish contact results for visualization.
  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  geometry::ConnectDrakeVisualizer(&builder, pair.scene_graph);
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  const VectorXd tau = VectorXd::Zero(plant.num_actuated_dofs());
  plant.get_actuation_input_port().FixValue(&plant_context, tau);

  // Set the body frame P initial pose.
  const Translation3d X_WP(0.0, 0.0, 0.95);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, body, X_WP);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_target_realtime_rate(0.75);
  simulator.Initialize();
  simulator.AdvanceTo(2.0);

  return 0;
}

}  // namespace
}  // namespace hsr
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  return drake::examples::hsr::main(argc, argv);
}
