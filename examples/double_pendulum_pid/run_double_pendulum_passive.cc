#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace double_pendulum {
namespace {

// using geometry::Capsule;
// using math::RigidTransformd;

static const char* const sdf_path = "drake/examples/double_pendulum_pid/models/double_pendulum.sdf";

void main() {

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double>& scene_graph = *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  multibody::MultibodyPlant<double>* dp =
      builder.AddSystem<multibody::MultibodyPlant<double>>(1.0e-3);
  dp->set_name("plant");
  dp->RegisterAsSourceForSceneGraph(&scene_graph);

  multibody::Parser parser(dp);
  const std::string dp_sdf_path = FindResourceOrThrow(sdf_path);
  multibody::ModelInstanceIndex plant_model_instance_index = parser.AddModelFromFile(dp_sdf_path);
  (void)plant_model_instance_index;

  // const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  // dp->RegisterVisualGeometry(
  //     dp->GetBodyByName("base"),
  //     /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
  //     RigidTransformd::Identity(), Capsule(2, 6), "visual", orange);

  // const auto& root_link = dp->GetBodyByName("base");
  // dp->AddJoint<multibody::WeldJoint>("weld_base", dp->world_body(), nullopt, root_link, nullopt, Isometry3<double>::Identity());
  dp->WeldFrames(dp->world_frame(), dp->GetFrameByName("base"));
  dp->Finalize();

  DRAKE_DEMAND(!!dp->get_source_id());
  builder.Connect(dp->get_geometry_poses_output_port(), scene_graph.get_source_pose_port(dp->get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(), dp->get_geometry_query_input_port());

  geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

  systems::Context<double>& plant_context = diagram->GetMutableSubsystemContext(*dp, diagram_context.get());

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.1;
  positions[1] = 0.4;
  dp->SetPositions(&plant_context, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(10);
}

}
}
}
}

int main() {
  drake::examples::double_pendulum::main();
  return 0;
}
