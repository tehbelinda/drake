#include "drake/geometry/render/gl_renderer/load_mesh.h"

#include <algorithm>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <tiny_obj_loader.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using std::string;
using std::vector;

std::tuple<VertexBuffer, IndexBuffer, NormalBuffer> LoadMeshFromObj(
    const string& filename) {
  tinyobj::attrib_t attrib;
  vector<tinyobj::shape_t> shapes;
  vector<tinyobj::material_t> materials;
  string err;
  // This renderer assumes everything is triangles -- we rely on tinyobj to
  // triangulate for us.
  bool do_tinyobj_triangulation = true;

  // Tinyobj doesn't infer the search directory from the directory containing
  // the obj file. We have to provide that directory; of course, this assumes
  // that the material library reference is relative to the obj directory.
  size_t pos = filename.find_last_of('/');
  const string obj_folder = filename.substr(0, pos + 1);
  const char* mtl_basedir = obj_folder.c_str();
  bool ret =
      tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename.c_str(),
                       mtl_basedir, do_tinyobj_triangulation);
  if (!ret || !err.empty()) {
    throw std::runtime_error(
        fmt::format("Error parsing file '{}': {}", filename, err));
  }

  DRAKE_DEMAND(shapes.size() > 0);
  // Set up buffer for vertices and normals.
  const vector<tinyobj::real_t>& verts = attrib.vertices;
  const int v_count = static_cast<int>(verts.size()) / 3;
  DRAKE_DEMAND(static_cast<int>(verts.size()) == v_count * 3);
  // We need to duplicate the vertices to account for different normals
  // depending on which triangle this vertex is being used for.
  const int duplicate_v_count = v_count * 3;
  VertexBuffer vertices{duplicate_v_count, 3};
  NormalBuffer normals{duplicate_v_count, 3};

  // Accumulate faces.
  int tri_count = 0;
  for (const auto& shape : shapes) {
    const tinyobj::mesh_t& raw_mesh = shape.mesh;
    DRAKE_DEMAND(raw_mesh.indices.size() ==
                 raw_mesh.num_face_vertices.size() * 3);
    tri_count += raw_mesh.num_face_vertices.size();
  }

  IndexBuffer indices{tri_count, 3};
  int tri_index = 0;
  for (const auto& shape : shapes) {
    const tinyobj::mesh_t& raw_mesh = shape.mesh;
    for (int f = 0; f < static_cast<int>(raw_mesh.num_face_vertices.size());
         ++f) {
      const tinyobj::index_t f0 = raw_mesh.indices[f * 3];
      const tinyobj::index_t f1 = raw_mesh.indices[f * 3 + 1];
      const tinyobj::index_t f2 = raw_mesh.indices[f * 3 + 2];
      const int v_f0 = f0.vertex_index;
      const int v_f1 = f1.vertex_index;
      const int v_f2 = f2.vertex_index;
      indices.block<1, 3>(tri_index, 0) << tri_index * 3, tri_index * 3 + 1,
          tri_index * 3 + 2;
      vertices.block<1, 3>(tri_index * 3, 0) << attrib.vertices[v_f0 * 3],
          attrib.vertices[v_f0 * 3 + 1], attrib.vertices[v_f0 * 3 + 2];
      vertices.block<1, 3>(tri_index * 3 + 1, 0) << attrib.vertices[v_f1 * 3],
          attrib.vertices[v_f1 * 3 + 1], attrib.vertices[v_f1 * 3 + 2];
      vertices.block<1, 3>(tri_index * 3 + 2, 0) << attrib.vertices[v_f2 * 3],
          attrib.vertices[v_f2 * 3 + 1], attrib.vertices[v_f2 * 3 + 2];
      if (attrib.normals.size() > 0) {
        const int n_f0 = f0.normal_index;
        const int n_f1 = f1.normal_index;
        const int n_f2 = f2.normal_index;
        normals.block<1, 3>(tri_index * 3, 0) << attrib.normals[n_f0 * 3],
            attrib.normals[n_f0 * 3 + 1], attrib.normals[n_f0 * 3 + 2];
        normals.block<1, 3>(tri_index * 3 + 1, 0)
            << attrib.normals[n_f1 * 3],
            attrib.normals[n_f1 * 3 + 1], attrib.normals[n_f1 * 3 + 2];
        normals.block<1, 3>(tri_index * 3 + 2, 0)
            << attrib.normals[n_f2 * 3],
            attrib.normals[n_f2 * 3 + 1], attrib.normals[n_f2 * 3 + 2];
      }

      ++tri_index;
    }
  }

  // NormalBuffer normals{v_count, 3};
  // if (attrib.normals.size() > 0) {
  //   // TODO(tehbelinda): Move this to the face loop to read in normals using
  //   // face.normal_index. Will need to add duplicate vertices to keep sharp
  //   // edges or calculate smooth normals across neighboring faces. Here is some
  //   // example usage from tinyobj:
  //   // https://github.com/tinyobjloader/tinyobjloader/blob/master/examples/viewer/viewer.cc
  //   // For now it's all just facing up.
  //   for (int v = 0; v < v_count; ++v) {
  //     normals.block<1, 3>(v, 0) << 0.f, 0.f, 1.f;
  //   }
  // }
  return std::make_tuple(vertices, indices, normals);
}

}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
