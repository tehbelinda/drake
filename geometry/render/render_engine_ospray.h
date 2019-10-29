#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include <vtkActor.h>
#include <vtkAutoInit.h>
#include <vtkImageExport.h>
#include <vtkLight.h>
#include <vtkNew.h>
#include <vtkOSPRayPass.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

#include "drake/geometry/render/render_engine_ospray_factory.h"
#include "drake/geometry/render/render_engine_vtk_base.h"

namespace drake {
namespace geometry {
namespace render {

/** See documentation for MakeRenderEngineOspray() for details.  */
class RenderEngineOspray final : public RenderEngineVtkBase {
 public:
  // TODO(SeanCurtis-TRI): Swap these shenanigans with a legitimate removal of
  //  all copy and move semantics. The current copy constructor's contents
  //  should simply go into the DoClone() method. The appropriate time to do
  //  this is with the VTK refactoring (resolving it here and for
  //  RenderEngineVtk). See issue #11964.
  /** @name Does not allow copy, move, or assignment  */
  //@{
#ifdef DRAKE_DOXYGEN_CXX
  // Note: the copy constructor is actually private to serve as the basis for
  // implementing the DoClone() method.
  RenderEngineOspray(const RenderEngineOspray&) = delete;
#endif
  RenderEngineOspray& operator=(const RenderEngineOspray&) = delete;
  RenderEngineOspray(RenderEngineOspray&&) = delete;
  RenderEngineOspray& operator=(RenderEngineOspray&&) = delete;
  //@}

  /** Constructs the render engine with the given `parameters`  */
  RenderEngineOspray(
      const RenderEngineOsprayParams& parameters = RenderEngineOsprayParams());

  /** @see RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const final;

  /** @name    Access the default properties

   Provides access to the default values this instance of the render engine is
   using. These values must be set at construction.  */
  //@{

  // TODO(SeanCurtis-TRI): Figure out a "default" material - material properties
  //  and its representation (this will evolve when I support full OSPRay
  //  materials).

  const Eigen::Vector4d& default_diffuse() const { return default_diffuse_; }

  const systems::sensors::ColorD& background_color() const {
    return background_color_;
  }

  //@}

 private:
  // Creates a copy of the set of parameters that were supplied when this
  // instance was created.
  RenderEngineOsprayParams get_params() const;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;

  // Copy constructor for the purpose of cloning.
  RenderEngineOspray(const RenderEngineOspray& other);

  // Initializes the VTK pipelines.
  void InitializePipelines(int samples_per_pixel);

  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(vtkPolyDataAlgorithm* source,
                         void* user_data) override;

  vtkNew<vtkLight> light_;

  // A single pipeline (for now): rgb.
  static constexpr int kNumPipelines = 1;

  std::array<std::unique_ptr<RenderingPipeline>, kNumPipelines> pipelines_;

  vtkNew<vtkOSPRayPass> ospray_;

  // The collection of per-geometry actors (one actor per pipeline (color,
  // depth, and label) keyed by the geometry's GeometryId.
  std::unordered_map<GeometryId,
                     std::array<vtkSmartPointer<vtkActor>, kNumPipelines>>
      actors_;

  // Color to assign to objects that define no color.
  Eigen::Vector4d default_diffuse_{0.9, 0.45, 0.1, 1.0};

  // The background color -- a sky blue.
  systems::sensors::ColorD background_color_{204 / 255., 229 / 255.,
                                             255 / 255.};

  // Configuration to use path tracer or ray tracer.
  const OsprayMode render_mode_{OsprayMode::kPathTracer};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
