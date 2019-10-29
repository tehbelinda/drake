#include "drake/geometry/render/render_engine_ospray.h"

#include <limits>
#include <stdexcept>
#include <utility>

// #include <vtkAppendPolyData.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkCylinderSource.h>
#include <vtkOBJReader.h>
#include <vtkOSPRayLightNode.h>
#include <vtkOSPRayMaterialLibrary.h>
#include <vtkOSPRayRendererNode.h>
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

using Eigen::Vector3d;
using Eigen::Vector4d;
using math::RigidTransformd;
using std::make_unique;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::vtk_util::ConvertToVtkTransform;
using systems::sensors::vtk_util::CreateSquarePlane;
using vtk_base::ImageType;
using vtk_base::RegistrationData;
using vtk_base::RemoveFileExtension;
using vtk_base::SetModelTransformMatrixToVtkCamera;

RenderEngineOspray::RenderEngineOspray(const RenderEngineOsprayParams& params)
    : RenderEngineVtkBase(RenderLabel::kUnspecified),
      pipelines_{{make_unique<RenderingPipeline>()}},
      render_mode_(params.mode) {
  if (params.default_diffuse) {
    default_diffuse_ = *params.default_diffuse;
  }

  if (params.background_color) {
    const Vector3d& c = *params.background_color;
    background_color_ = ColorD{c(0), c(1), c(2)};
  }

  InitializePipelines(params.samples_per_pixel);
}

void RenderEngineOspray::RenderColorImage(const CameraProperties& camera,
                                          bool show_window,
                                          ImageRgba8U* color_image_out) const {
  UpdateWindow(camera, show_window, pipelines_[ImageType::kColor].get(),
               "Color Image");
  PerformVtkUpdate(*pipelines_[ImageType::kColor]);

  // TODO(SeanCurtis-TRI): Determine if this copies memory (and find some way
  // around copying).
  auto& exporter = *pipelines_[ImageType::kColor]->exporter;
  DRAKE_DEMAND(exporter.GetDataNumberOfScalarComponents() == 4);
  exporter.Export(color_image_out->at(0, 0));

  // In path tracing, the background contributes light energy to the rendering.
  // However, the raytracer and gl renderers don't do this, it's simply a
  // backdrop -- a color for pixels that weren't otherwise covered by geometry.
  // We simulate that effect by blending the final pixels with the background
  // color.
  // TODO(SeanCurtis-TRI): Make this configurable; i.e., if we provide an hdmi
  //  environment map, this shouldn't happen at all.
  if (render_mode_ == OsprayMode::kPathTracer) {
    using ChannelType = ImageRgba8U::Traits::ChannelType;
    // Note: the cast truncates, by adding 0.5, it becomes rounding.
    systems::sensors::ColorI rgb{
        static_cast<ChannelType>(background_color_.r * 255 + 0.5),
        static_cast<ChannelType>(background_color_.g * 255 + 0.5),
        static_cast<ChannelType>(background_color_.b * 255 + 0.5)};

    auto blend = [](ImageRgba8U::Traits::ChannelType channel,
                    ImageRgba8U::Traits::ChannelType bg, double alpha) {
      double combo = channel * alpha + bg * (1 - alpha);
      return static_cast<ImageRgba8U::Traits::ChannelType>(combo);
    };

    for (int r = 0; r < color_image_out->height(); ++r) {
      for (int c = 0; c < color_image_out->width(); ++c) {
        ChannelType* pixel = color_image_out->at(c, r);
        double alpha = pixel[3] / 255.0;
        if (alpha < 1.0) {
          pixel[0] = blend(pixel[0], rgb.r, alpha);
          pixel[1] = blend(pixel[1], rgb.g, alpha);
          pixel[2] = blend(pixel[2], rgb.b, alpha);
        }
      }
    }
  }
}

void RenderEngineOspray::RenderDepthImage(const DepthCameraProperties&,
                                          ImageDepth32F*) const {
  throw std::runtime_error("RenderEngineOspray does not support depth images");
}

void RenderEngineOspray::RenderLabelImage(const CameraProperties&, bool,
                                       ImageLabel16I*) const {
  throw std::runtime_error("RenderEngineOspray does not support label images");
}

RenderEngineOsprayParams RenderEngineOspray::get_params() const {
  const int samples_per_pixel = vtkOSPRayRendererNode::GetSamplesPerPixel(
      pipelines_[ImageType::kColor]->renderer);
  return {
      render_mode_, default_diffuse_,
      Vector3d{background_color_.r, background_color_.g, background_color_.b},
      samples_per_pixel};
}

std::unique_ptr<RenderEngine> RenderEngineOspray::DoClone() const {
  // Note: we can't use make_unique because the copy constructor is private.
  return std::unique_ptr<RenderEngineOspray>(new RenderEngineOspray(*this));
}

// Note: this is a private copy constructor implemented solely to facilitate
// cloning. This code should simply be rolled into DoClone(). (See the TODO
// in the header file.)
RenderEngineOspray::RenderEngineOspray(const RenderEngineOspray& other)
    : RenderEngineVtkBase(other),
      pipelines_{{make_unique<RenderingPipeline>()}},
      default_diffuse_{other.default_diffuse_},
      background_color_{other.background_color_},
      render_mode_(other.render_mode_) {
  InitializePipelines(other.get_params().samples_per_pixel);

  // Utility function for creating a cloned actor which *shares* the same
  // underlying polygonal data.
  auto clone_actor_array =
      [this](const std::array<vtkSmartPointer<vtkActor>, kNumPipelines>&
                 source_actors,
             std::array<vtkSmartPointer<vtkActor>, kNumPipelines>*
                 clone_actors_ptr) {
        DRAKE_DEMAND(clone_actors_ptr != nullptr);
        std::array<vtkSmartPointer<vtkActor>, kNumPipelines>& clone_actors =
            *clone_actors_ptr;
        for (int i = 0; i < kNumPipelines; ++i) {
          // NOTE: source *should* be const; but none of the getters on the
          // source are const-compatible.
          DRAKE_DEMAND(source_actors[i]);
          DRAKE_DEMAND(clone_actors[i]);
          vtkActor& source = *source_actors[i];
          vtkActor& clone = *clone_actors[i];

          // TODO(SeanCurtis-TRI): Modify this once OSPRay-specific materials
          //  are supported.
          if (source.GetTexture() == nullptr) {
            clone.GetProperty()->SetColor(source.GetProperty()->GetColor());
            clone.GetProperty()->SetOpacity(source.GetProperty()->GetOpacity());
          } else {
            clone.SetTexture(source.GetTexture());
          }

          // NOTE: The clone renderer and original renderer *share* polygon
          // data. If the meshes were *deformable* this would be invalid.
          // Furthermore, even if dynamic adding/removing of geometry were
          // valid, VTK's reference counting preserves the underlying geometry
          // in the copy that still references it.
          clone.SetMapper(source.GetMapper());
          clone.SetUserTransform(source.GetUserTransform());

          pipelines_.at(i)->renderer.Get()->AddActor(&clone);
        }
      };

  for (const auto& other_id_actor_pair : other.actors_) {
    std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
        vtkSmartPointer<vtkActor>::New()};
    clone_actor_array(other_id_actor_pair.second, &actors);
    const GeometryId id = other_id_actor_pair.first;
    actors_.insert({id, move(actors)});
  }

  // Copy camera properties
  auto copy_cameras = [](auto src_renderer, auto dst_renderer) {
    dst_renderer->GetActiveCamera()->DeepCopy(src_renderer->GetActiveCamera());
  };
  for (int p = 0; p < kNumPipelines; ++p) {
    copy_cameras(other.pipelines_.at(p)->renderer.Get(),
                 pipelines_.at(p)->renderer.Get());
  }

  // TODO(SeanCurtis-TRI): Copy light.
}

void RenderEngineOspray::InitializePipelines(int samples_per_pixel) {
  const vtkSmartPointer<vtkTransform> vtk_identity =
      ConvertToVtkTransform(RigidTransformd::Identity());

  // TODO(SeanCurtis-TRI): Things like configuring lights should *not* be part
  //  of initializing the pipelines. When we support light declaration, this
  //  will get moved out.
  light_->SetLightTypeToCameraLight();
  light_->SetConeAngle(45.0);
  light_->SetAttenuationValues(1.0, 0.0, 0.0);
  light_->SetIntensity(1);
  // OSPRay specific control, radius to get soft shadows.
  if (render_mode_ == OsprayMode::kPathTracer) {
    vtkOSPRayLightNode::SetRadius(1.0, light_);
  }
  light_->SetTransformMatrix(vtk_identity->GetMatrix());

  // Generic configuration of pipelines.
  for (auto& pipeline : pipelines_) {
    // OSPRay specific configuration.
    pipeline->renderer->SetPass(ospray_);
    if (render_mode_ == OsprayMode::kRayTracer) {
      vtkOSPRayRendererNode::SetRendererType("scivis", pipeline->renderer);
      vtkOSPRayRendererNode::SetAmbientSamples(0, pipeline->renderer);
      pipeline->renderer->UseShadowsOn();
      // NOTE: It appears that ospray does [0, 1] -> [0, 255] conversion via
      // truncation. So, to affect rounding, we have to bump the background
      // color by half a bit so that it rounds properly.
      const double delta = 0.5 / 255;
      ColorD bg{background_color_.r + delta, background_color_.g + delta,
                background_color_.b + delta};
      pipeline->renderer->SetBackground(bg.r, bg.g, bg.b);
    } else {
      vtkOSPRayRendererNode::SetRendererType("pathtracer", pipeline->renderer);
      vtkOSPRayRendererNode::SetSamplesPerPixel(samples_per_pixel,
                                                pipeline->renderer);
      // TODO(SeanCurtis-TRI): When our VTK library has been updated to include
      //  the denoiser introduced in
      //  https://gitlab.kitware.com/vtk/vtk/merge_requests/5297
      //  make the denoiser and its threshold parameter available.
      //  vtkOSPRayRendererNode::ENABLE_DENOISER();
      //  vtkOSPRayRendererNode::SetEnableDenoiser(4, pipeline->renderer);
    }
    double np[3] = {0, 0, 1};
    double ep[3] = {0, 1, 0};
    vtkOSPRayRendererNode::SetNorthPole(np, pipeline->renderer);
    vtkOSPRayRendererNode::SetEastPole(ep, pipeline->renderer);

    auto camera = pipeline->renderer->GetActiveCamera();
    camera->SetViewAngle(90.0);  // Default to an arbitrary 90° field of view.
    SetModelTransformMatrixToVtkCamera(camera, vtk_identity);

    pipeline->window->AddRenderer(pipeline->renderer.GetPointer());
    pipeline->filter->SetInput(pipeline->window.GetPointer());
    pipeline->filter->SetScale(1);
    pipeline->filter->ReadFrontBufferOff();
    pipeline->filter->SetInputBufferTypeToRGBA();
    pipeline->exporter->SetInputData(pipeline->filter->GetOutput());
    pipeline->exporter->ImageLowerLeftOff();

    pipeline->renderer->AddLight(light_);
  }
}

void RenderEngineOspray::ImplementGeometry(vtkPolyDataAlgorithm* source,
                                           void* user_data) {
  DRAKE_DEMAND(user_data != nullptr);

  std::array<vtkSmartPointer<vtkActor>, kNumPipelines> actors{
      vtkSmartPointer<vtkActor>::New()};
  // Note: the mappers ultimately get referenced by the actors, so they do _not_
  // get destroyed when this array goes out of scope.
  std::array<vtkNew<vtkOpenGLPolyDataMapper>, kNumPipelines> mappers;

  for (auto& mapper : mappers) {
    mapper->SetInputConnection(source->GetOutputPort());
  }

  const RegistrationData& data =
      *reinterpret_cast<RegistrationData*>(user_data);

  // If the geometry is anchored, X_FG = X_WG so I'm setting the pose for
  // anchored geometry -- for all other values of F, it is dynamic and will be
  // re-written in the first pose update.
  vtkSmartPointer<vtkTransform> vtk_X_PG = ConvertToVtkTransform(data.X_FG);

  // Adds the actor into the specified pipeline.
  auto connect_actor = [this, &actors, &mappers,
      &vtk_X_PG](ImageType image_type) {
    actors[image_type]->SetMapper(mappers[image_type].Get());
    actors[image_type]->SetUserTransform(vtk_X_PG);
    pipelines_[image_type]->renderer->AddActor(actors[image_type].Get());
  };

  // Color actor.
  auto& color_actor = actors[ImageType::kColor];

  // TODO(SeanCurtis-TRI): Modify this once OSPRay-specific materials are
  //  supported.
  const std::string& diffuse_map_name =
      data.properties.GetPropertyOrDefault<std::string>("phong", "diffuse_map",
                                                        "");
  // Legacy support for *implied* texture maps. If we have mesh.obj, we look for
  // mesh.png (unless one has been specifically called out in the properties).
  // TODO(SeanCurtis-TRI): Remove this legacy texture when objects and materials
  // are coherently specified by SDF/URDF/obj/mtl, etc.
  std::string texture_name;
  std::ifstream file_exist(diffuse_map_name);
  if (file_exist) {
    texture_name = diffuse_map_name;
  } else if (diffuse_map_name.empty() && data.mesh_filename) {
    // This is the hack to search for mesh.png as a possible texture.
    const std::string alt_texture_name(
        RemoveFileExtension(*data.mesh_filename) + ".png");
    std::ifstream alt_file_exist(alt_texture_name);
    if (alt_file_exist) texture_name = alt_texture_name;
  }
  if (!texture_name.empty()) {
    vtkNew<vtkPNGReader> texture_reader;
    texture_reader->SetFileName(texture_name.c_str());
    texture_reader->Update();
    vtkNew<vtkOpenGLTexture> texture;
    texture->SetInputConnection(texture_reader->GetOutputPort());
    texture->InterpolateOn();
    color_actor->SetTexture(texture.Get());
  } else {
    const Vector4d& diffuse =
        data.properties.GetPropertyOrDefault("phong", "diffuse",
                                             default_diffuse_);
    color_actor->GetProperty()->SetColor(diffuse(0), diffuse(1), diffuse(2));
    color_actor->GetProperty()->SetOpacity(diffuse(3));
  }

  connect_actor(ImageType::kColor);

  // Take ownership of the actors.
  actors_.insert({data.id, std::move(actors)});
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
