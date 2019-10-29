#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkCylinderSource.h>
#include <vtkImageExport.h>
#include <vtkNew.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkShaderProgram.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkWindowToImageFilter.h>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"

#ifndef DRAKE_DOXYGEN_CXX
// This, and the ModuleInitVtkRenderingOpenGL2, provide the basis for enabling
// VTK's OpenGL2 infrastructure.
VTK_AUTOINIT_DECLARE(vtkRenderingOpenGL2)
#endif

namespace drake {
namespace geometry {
namespace render {

namespace vtk_base {
using math::RigidTransformd;
using systems::sensors::InvalidDepth;

// A note on OpenGL clipping planes and DepthCameraProperties near and far.
// The z near and far planes reported in DepthCameraProperties are properties of
// the modeled sensor. They cannot produce reliable depths closer than z_near
// or farther than z_far.
// We *could* set the OpenGl clipping planes to [z_near, z_far], however, that
// will lead to undesirable artifacts. Any geometry that lies at a distance
// in the range [0, z_near) would be clipped away with *no* occluding effects.
// So, instead, we render the depth image in the range [epsilon, z_far] and
// then detect the occluding pixels that lie closer than the camera's supported
// range and mark them as too close. Clipping all geometry beyond z_far is not
// a problem because they can unambiguously be marked as too far.
const double kClippingPlaneNear = 0.01;
const double kTerrainSize = 100.;

void SetModelTransformMatrixToVtkCamera(
    vtkCamera* camera, const vtkSmartPointer<vtkTransform>& X_WC);

float CheckRangeAndConvertToMeters(float z_buffer_value, double z_near,
                                   double z_far);

enum ImageType {
  kColor = 0,
  kLabel = 1,
  kDepth = 2,
};

// TODO(SeanCurtis-TRI): Add X_PG pose to this data.
// A package of data required to register a visual geometry.
struct RegistrationData {
  const PerceptionProperties& properties;
  const RigidTransformd& X_FG;
  const GeometryId id;
  // The file name if the shape being registered is a mesh.
  optional<std::string> mesh_filename;
};

std::string RemoveFileExtension(const std::string& filepath);

}  // namespace vtk_base

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
struct ModuleInitVtkRenderingOpenGL2 {
  ModuleInitVtkRenderingOpenGL2(){
    VTK_AUTOINIT_CONSTRUCT(vtkRenderingOpenGL2)
  }
};

}  // namespace internal

#endif  // !DRAKE_DOXYGEN_CXX

/** Base class for VTK based render engines.  */
class RenderEngineVtkBase : public RenderEngine,
                            protected internal::ModuleInitVtkRenderingOpenGL2 {
 public:
  virtual ~RenderEngineVtkBase() = default;

  /** Constructs the render engine from the given `parameters`.

   When one of the optional parameters is omitted, the constructed value will be
   as documented elsewhere in @ref render_engine_vtk_properties "this class".
  */
  explicit RenderEngineVtkBase(
      const RenderLabel& default_label = RenderLabel::kUnspecified)
      : RenderEngine(default_label) {}

  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @name    Shape reification  */
  //@{
  void ImplementGeometry(const Sphere& sphere, void* user_data) final;
  void ImplementGeometry(const Cylinder& cylinder, void* user_data) final;
  void ImplementGeometry(const HalfSpace& half_space, void* user_data) final;
  void ImplementGeometry(const Box& box, void* user_data) final;
  void ImplementGeometry(const Capsule& capsule, void* user_data) final;
  void ImplementGeometry(const Mesh& mesh, void* user_data) final;
  void ImplementGeometry(const Convex& convex, void* user_data) final;
  //@}

 protected:
  RenderEngineVtkBase(const RenderEngineVtkBase& other) : RenderEngine(other) {}

  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(
      GeometryId id, const Shape& shape, const PerceptionProperties& properties,
      const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // Common interface for loading an obj file -- used for both mesh and convex
  // shapes.
  void ImplementObj(const std::string& file_name, double scale,
                    void* user_data);

  // Performs the common setup for all shape types.
  virtual void ImplementGeometry(vtkPolyDataAlgorithm* source,
                                 void* user_data) = 0;

  // The rendering pipeline for a single image type (color, depth, or label).
  struct RenderingPipeline {
    vtkNew<vtkRenderer> renderer;
    vtkNew<vtkRenderWindow> window;
    vtkNew<vtkWindowToImageFilter> filter;
    vtkNew<vtkImageExport> exporter;
  };

  // Updates VTK rendering related objects including vtkRenderWindow,
  // vtkWindowToImageFilter and vtkImageExporter, so that VTK reflects
  // vtkActors' pose update for rendering.
  static void PerformVtkUpdate(const RenderingPipeline& p);

  // This actually modifies internal state; the pointer to a const pipeline
  // allows mutation via the contained vtkNew pointers.
  virtual void UpdateWindow(const CameraProperties& camera, bool show_window,
                            const RenderingPipeline* p, const char* name) const;

  // Three pipelines: rgb, depth, and label.
  static constexpr int kNumPipelines = 3;

  std::array<std::unique_ptr<RenderingPipeline>, kNumPipelines> pipelines_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId, std::array<vtkSmartPointer<vtkActor>, 3>>
      actors_;

  void SetSphereOptions(vtkSphereSource* vtk_sphere, double radius);
  void SetCylinderOptions(vtkCylinderSource* vtk_cylinder, double height,
                          double radius);
  void TransformToDrakeCylinder(vtkTransform* transform,
                                vtkTransformPolyDataFilter* transform_filter,
                                vtkCylinderSource* vtk_cylinder);
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
