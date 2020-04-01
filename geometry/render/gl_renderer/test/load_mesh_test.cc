#include "drake/geometry/render/gl_renderer/load_mesh.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {
namespace {

// Tests loading a mesh from an .obj file. It relies on the specific content of
// the file quad_cube.obj.
GTEST_TEST(LoadMesh, LoadObjMesh) {
  auto filename =
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj");
  auto [vertices, indices, normals] = LoadMeshFromObj(filename);

  // These coordinates came from the first section of quad_cube.obj.
  // clang-format off
  std::vector<Vector3<float>> expect_vertices {
      { 1.000000, -1.000000, -1.000000},
      { 1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000,  1.000000},
      {-1.000000, -1.000000, -1.000000},
      { 1.000000,  1.000000, -1.000000},
      { 1.000000,  1.000000,  1.000001},
      {-1.000000,  1.000000,  1.000000},
      {-1.000000,  1.000000, -1.000000}
  };
  // clang-format on

  for (int i = 0; i < 8; ++i) {
    Vector3<float> vertex{vertices(i * 3), vertices(i * 3 + 1),
                          vertices(i * 3 + 2)};
    EXPECT_EQ(expect_vertices[i], vertex);
  }

  // The last section of quad_cube.obj describes the six square faces of the
  // cube. We expect that each square is subdivided into two triangles. Note
  // that vertex indices in the file quad_cube.obj are from 1 to 8, but tinyobj
  // has vertex indices from 0 to 7. We assume that tinyobj subdivides a
  // polygon into a triangle fan around the first vertex; polygon ABCDE...
  // becomes triangles ABC, ACD, ADE, etc.
  int expect_faces[12][3]{
      {0, 1, 2}, {0, 2, 3},  // face 1 2 3 4 in quad_cube.obj
      {4, 7, 6}, {4, 6, 5},  // face 5 8 7 6 in quad_cube.obj
      {0, 4, 5}, {0, 5, 1},  // face 1 5 6 2 in quad_cube.obj
      {1, 5, 6}, {1, 6, 2},  // face 2 6 7 3 in quad_cube.obj
      {2, 6, 7}, {2, 7, 3},  // face 3 7 8 4 in quad_cube.obj
      {4, 0, 3}, {4, 3, 7}   // face 5 1 4 8 in quad_cube.obj
  };
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(expect_faces[i][j], indices(i * 3 + j));
    }
  }
}

// The middle section of quad_cube.obj describes the normals. Similar to the
// faces above, we assume the normals for subdivided triangles.

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
