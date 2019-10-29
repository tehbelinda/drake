#include "drake/geometry/render/render_engine_vtk_base.h"

#include <limits>
#include <stdexcept>
#include <utility>

#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLTexture.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>

#include "drake/common/text_logging.h"
#include "drake/systems/sensors/color_palette.h"
#include "drake/systems/sensors/vtk_util.h"

namespace drake {
namespace geometry {
namespace render {

namespace vtk_base {

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC) {
  // vtkCamera contains a transformation as the internal state and
  // ApplyTransform multiplies a given transformation on top of the internal
  // transformation. Thus, resetting 'Set{Position, FocalPoint, ViewUp}' is
  // needed here.
  camera->SetPosition(0., 0., 0.);
  // Sets z-forward.
  camera->SetFocalPoint(0., 0., 1.);
  // Sets y-down. For the detail, please refer to CameraInfo's document.
  camera->SetViewUp(0., -1, 0.);
  camera->ApplyTransform(X_WC);
}

float CheckRangeAndConvertToMeters(float z_buffer_value, double z_near,
                                   double z_far) {
  const double kA = z_far - kClippingPlaneNear;
  // Initialize with the assumption that the buffer value is outside the range
  // [kClippingPlaneNear, z_far]. If the buffer value is *not* 1,
  // then it lies inside the range.
  float z = std::numeric_limits<float>::quiet_NaN();
  if (z_buffer_value != 1.f) {
    z = static_cast<float>(z_buffer_value * kA + kClippingPlaneNear);
    // TODO(SeanCurtis-TRI): This is now slightly strange; the OpenGL clipping
    //  plane is being set to the camera's z_far; we should never get a value
    //  greater than that. Clean this up. Ideally, do more in the shader.
    if (z > z_far) {
      z = InvalidDepth::kTooFar;
    } else if (z < z_near) {
      z = InvalidDepth::kTooClose;
    }
  }
  return z;
}

std::string RemoveFileExtension(const std::string& filepath) {
  const size_t last_dot = filepath.find_last_of(".");
  if (last_dot == std::string::npos) {
    throw std::logic_error("File has no extension.");
  }
  return filepath.substr(0, last_dot);
}

}  // namespace vtk_base

using Eigen::Vector4d;
using std::make_unique;
using math::RigidTransformd;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::vtk_util::ConvertToVtkTransform;
using systems::sensors::vtk_util::CreateSquarePlane;
using systems::sensors::vtk_util::MakeVtkPointerArray;
using vtk_base::kClippingPlaneNear;
using vtk_base::kTerrainSize;
using vtk_base::ImageType;
using vtk_base::RegistrationData;
using vtk_base::SetModelTransformMatrixToVtkCamera;

void RenderEngineVtkBase::UpdateViewpoint(const RigidTransformd& X_WC) {
  vtkSmartPointer<vtkTransform> vtk_X_WC = ConvertToVtkTransform(X_WC);

  for (const auto& pipeline : pipelines_) {
    auto camera = pipeline->renderer->GetActiveCamera();
    SetModelTransformMatrixToVtkCamera(camera, vtk_X_WC);
  }
}

void RenderEngineVtkBase::ImplementGeometry(const Sphere& sphere,
                                            void* user_data) {
  // TODO(SeanCurtis-TRI): OSPRay supports a primitive sphere; find some way to
  //  exercise *that* instead of needlessly tessellating.
  vtkNew<vtkSphereSource> vtk_sphere;
  SetSphereOptions(vtk_sphere.GetPointer(), sphere.get_radius());
  ImplementGeometry(vtk_sphere.GetPointer(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const Cylinder& cylinder,
                                            void* user_data) {
  // TODO(SeanCurtis-TRI): OSPRay supports a primitive cylinder; find some way
  //  to exercise *that* instead of needlessly tessellating.
  vtkNew<vtkCylinderSource> vtk_cylinder;
  SetCylinderOptions(vtk_cylinder, cylinder.get_length(),
                     cylinder.get_radius());

  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  TransformToDrakeCylinder(transform, transform_filter, vtk_cylinder);

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const HalfSpace&,
                                        void* user_data) {
  vtkSmartPointer<vtkPlaneSource> vtk_plane = CreateSquarePlane(kTerrainSize);

  ImplementGeometry(vtk_plane.GetPointer(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const Box& box, void* user_data) {
  vtkNew<vtkCubeSource> cube;
  cube->SetXLength(box.width());
  cube->SetYLength(box.depth());
  cube->SetZLength(box.height());
  ImplementGeometry(cube.GetPointer(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const Capsule& capsule,
                                            void* user_data) {
  vtkNew<vtkCylinderSource> vtk_cylinder;
  SetCylinderOptions(vtk_cylinder, capsule.get_length(), capsule.get_radius());
  // Since the cylinder in vtkCylinderSource is y-axis aligned, we need
  // to rotate it to be z-axis aligned because that is what Drake uses.
  vtkNew<vtkTransform> transform;
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  TransformToDrakeCylinder(transform, transform_filter, vtk_cylinder);
  ImplementGeometry(transform_filter.GetPointer(), user_data);

  vtkNew<vtkSphereSource> vtk_sphere_a;
  SetSphereOptions(vtk_sphere_a.GetPointer(), capsule.get_radius());
  ImplementGeometry(vtk_sphere_a.GetPointer(), user_data);

  vtkNew<vtkSphereSource> vtk_sphere_b;
  SetSphereOptions(vtk_sphere_b.GetPointer(), capsule.get_radius());
  ImplementGeometry(vtk_sphere_b.GetPointer(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const Mesh& mesh, void* user_data) {
  ImplementObj(mesh.filename(), mesh.scale(), user_data);
}

void RenderEngineVtkBase::ImplementGeometry(const Convex& convex,
                                            void* user_data) {
  ImplementObj(convex.filename(), convex.scale(), user_data);
}

bool RenderEngineVtkBase::DoRegisterVisual(
    GeometryId id, const Shape& shape, const PerceptionProperties& properties,
    const RigidTransformd& X_FG) {
  // Note: the user_data interface on reification requires a non-const pointer.
  RegistrationData data{properties, X_FG, id};
  shape.Reify(this, &data);
  return true;
}

void RenderEngineVtkBase::DoUpdateVisualPose(GeometryId id,
                                         const RigidTransformd& X_WG) {
  vtkSmartPointer<vtkTransform> vtk_X_WG = ConvertToVtkTransform(X_WG);
  // TODO(SeanCurtis-TRI): Perhaps provide the ability to specify actors for
  //  specific pipelines; i.e. only update the color actor or only the label
  //  actor, etc.
  for (const auto& actor : actors_.at(id)) {
    actor->SetUserTransform(vtk_X_WG);
  }
}

bool RenderEngineVtkBase::DoRemoveGeometry(GeometryId id) {
  auto iter = actors_.find(id);

  if (iter == actors_.end()) return false;

  std::array<vtkSmartPointer<vtkActor>, 3>& pipe_actors = iter->second;
  for (int i = 0; i < kNumPipelines; ++i) {
    // If the label actor hasn't been added to its renderer, this is a no-op.
    pipelines_[i]->renderer->RemoveActor(pipe_actors[i]);
  }
  actors_.erase(iter);
  return true;
}

void RenderEngineVtkBase::ImplementObj(const std::string& file_name,
                                       double scale, void* user_data) {
  static_cast<RegistrationData*>(user_data)->mesh_filename = file_name;
  vtkNew<vtkOBJReader> mesh_reader;
  mesh_reader->SetFileName(file_name.c_str());
  mesh_reader->Update();

  vtkNew<vtkTransform> transform;
  // TODO(SeanCurtis-TRI): Should I be allowing only isotropic scale.
  // TODO(SeanCurtis-TRI): Only add the transform filter if scale is not all 1.
  transform->Scale(scale, scale, scale);
  vtkNew<vtkTransformPolyDataFilter> transform_filter;
  transform_filter->SetInputConnection(mesh_reader->GetOutputPort());
  transform_filter->SetTransform(transform.GetPointer());
  transform_filter->Update();

  ImplementGeometry(transform_filter.GetPointer(), user_data);
}

void RenderEngineVtkBase::PerformVtkUpdate(const RenderingPipeline& p) {
  p.window->Render();
  // See the note in the VTK documentation about explicitly calling Modified
  // on the filter:
  // https://vtk.org/doc/nightly/html/classvtkWindowToImageFilter.html#details
  p.filter->Modified();
  p.filter->Update();
}

void RenderEngineVtkBase::UpdateWindow(const CameraProperties& camera,
                                       bool show_window,
                                       const RenderingPipeline* p,
                                       const char* name) const {
  // NOTE: This is a horrible hack for modifying what otherwise looks like
  // const entities.
  p->window->SetSize(camera.width, camera.height);
  p->window->SetOffScreenRendering(!show_window);
  if (show_window) p->window->SetWindowName(name);
  p->renderer->GetActiveCamera()->SetViewAngle(camera.fov_y * 180 / M_PI);
}

void RenderEngineVtkBase::SetSphereOptions(vtkSphereSource* vtk_sphere,
                                           double radius) {
  vtk_sphere->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_sphere->SetThetaResolution(50);
  vtk_sphere->SetPhiResolution(50);
}

void RenderEngineVtkBase::SetCylinderOptions(vtkCylinderSource* vtk_cylinder,
                                            double height, double radius) {
  vtk_cylinder->SetHeight(height);
  vtk_cylinder->SetRadius(radius);
  // TODO(SeanCurtis-TRI): Provide control for smoothness/tessellation.
  vtk_cylinder->SetResolution(50);
}

void RenderEngineVtkBase::TransformToDrakeCylinder(
    vtkTransform* transform, vtkTransformPolyDataFilter* transform_filter,
    vtkCylinderSource* vtk_cylinder) {
  transform->RotateX(90);
  transform_filter->SetInputConnection(vtk_cylinder->GetOutputPort());
  transform_filter->SetTransform(transform);
  transform_filter->Update();
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
