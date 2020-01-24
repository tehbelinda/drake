#include "drake/geometry/proximity/bounding_volume_hierarchy.h"

namespace drake {
namespace geometry {
namespace internal {

bool Aabb::HasOverlap(const Aabb& a, const Aabb& b,
                      const math::RigidTransform<double>& X_AB) {
  // We need to split the transform into the position and rotation components,
  // `p_AB` and `R_AB`. For the purposes of streamlining the math below, they
  // will henceforth be named `t` and `r` respectively.
  const Vector3<double> t = X_AB * b.center() - a.center();
  const Matrix3<double> r = X_AB.rotation().matrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel. We use the value as
  // specified from Gottschalk's OBB robustness tests.
  const double kEpsilon = 0.000001;
  Matrix3<double> abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r(i, j) = abs(abs_r(i, j)) + kEpsilon;
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t[i]) >
        a.half_width()[i] + b.half_width().dot(abs_r.block<1, 3>(i, 0))) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    if (abs(t.dot(r.block<3, 1>(0, i))) >
        b.half_width()[i] + a.half_width().dot(abs_r.block<3, 1>(0, i))) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (abs(t[i2] * r(i1, j) -
              t[i1] * r(i2, j)) >
          a.half_width()[i1] * abs_r(i2, j) +
              a.half_width()[i2] * abs_r(i1, j) +
              b.half_width()[j1] * abs_r(i, j2) +
              b.half_width()[j2] * abs_r(i, j1)) {
        return false;
      }
      j1 = j2;
    }
    i1 = i2;
  }

  return true;
}

void Aabb::PadBoundary() {
  const double max_position = center_.cwiseAbs().maxCoeff();
  const double max_half_width = half_width_.maxCoeff();
  const double scale = std::max(max_position, max_half_width);
  const double incr =
      std::max(scale * std::numeric_limits<double>::epsilon(), kTolerance);
  half_width_ += Vector3<double>::Constant(incr);
}

template <class MeshType>
BoundingVolumeHierarchy<MeshType>::BoundingVolumeHierarchy(
    const MeshType& mesh, bool h) {
  // Generate element indices and corresponding centroids. These are used
  // for calculating the split point of the volumes.
  const int num_elements = mesh.num_elements();
  std::vector<CentroidPair> element_centroids;
  for (IndexType i(0); i < num_elements; ++i) {
    element_centroids.emplace_back(i, ComputeCentroid(mesh, i));
  }

  root_node_ =
      BuildBVTree(mesh, element_centroids.begin(), element_centroids.end(), h);
}

template <class MeshType>
std::unique_ptr<BvNode<MeshType>>
BoundingVolumeHierarchy<MeshType>::BuildBVTree(
    const MeshType& mesh,
    const typename std::vector<CentroidPair>::iterator& start,
    const typename std::vector<CentroidPair>::iterator& end, bool h) {
  // Generate bounding volume.
  Aabb aabb = ComputeBoundingVolume(mesh, start, end);

  const int num_elements = end - start;
  if (num_elements == 1) {
    // Store element index in this leaf node.
    return std::make_unique<BvNode<MeshType>>(aabb, start->first);

  } else {
    // We want to use a volume based heuristic to find the optimal splitting
    // point. First we sort the elements by their centroid. Then we compute the
    // cost by summing the volume of the two child bounding volumes formed at
    // each interval. We repeat this for each axis to find the minimum cost
    // across all of them.
    int axis{};
    aabb.half_width().maxCoeff(&axis);

    double optimal_axis = axis;
    std::sort(start, end,
              [optimal_axis](const CentroidPair& a, const CentroidPair& b) {
                return a.second[optimal_axis] < b.second[optimal_axis];
              });

    typename std::vector<CentroidPair>::iterator optimal_split;
    if (h) {
      double optimal_volume = std::numeric_limits<double>::max();
      // for (int axis = 0; axis < 3; ++axis) {
        // std::sort(start, end,
        //           [axis](const CentroidPair& a, const CentroidPair& b) {
        //             return a.second[axis] < b.second[axis];
        //           });
        for (typename std::vector<CentroidPair>::iterator split = start + 1;
            split < end; ++split) {
          const double child_volume =
              ComputeBoundingVolume(mesh, start, split).CalcVolume() +
              ComputeBoundingVolume(mesh, split, end).CalcVolume();
          if (child_volume < optimal_volume) {
            optimal_volume = child_volume;
            optimal_split = split;
          }
        }
      // }
      // std::cout << "Optimal split is " << (optimal_split - start)  << " out of " << num_elements << " elements" << std::endl;
    } else {
      // int axis{};
      // aabb.half_width().maxCoeff(&axis);
      // double optimal_axis = axis;
      // std::sort(start, end,
      //           [optimal_axis](const CentroidPair& a, const CentroidPair& b) {
      //             return a.second[optimal_axis] < b.second[optimal_axis];
      //           });
      optimal_split = start + num_elements / 2;
    }

    // Continue with the next branches.
    return std::make_unique<BvNode<MeshType>>(
        aabb, BuildBVTree(mesh, start, optimal_split, h),
        BuildBVTree(mesh, optimal_split, end, h));
  }
}

template <class MeshType>
Aabb BoundingVolumeHierarchy<MeshType>::ComputeBoundingVolume(
    const MeshType& mesh,
    const typename std::vector<CentroidPair>::iterator& start,
    const typename std::vector<CentroidPair>::iterator& end) {
  // Keep track of the min/max bounds to create the bounding box.
  Vector3<double> max_bounds, min_bounds;
  max_bounds.setConstant(std::numeric_limits<double>::lowest());
  min_bounds.setConstant(std::numeric_limits<double>::max());

  // Check each mesh element in the given range.
  for (auto pair = start; pair < end; ++pair) {
    const auto& element = mesh.element(pair->first);
    // Check each vertex in the element.
    for (int v = 0; v < kElementVertexCount; ++v) {
      const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
      // Compare its extent along each of the 3 axes.
      min_bounds = min_bounds.cwiseMin(vertex);
      max_bounds = max_bounds.cwiseMax(vertex);
    }
  }
  const Vector3<double> center = (min_bounds + max_bounds) / 2;
  const Vector3<double> half_width = max_bounds - center;
  return Aabb(center, half_width);
}

template <class MeshType>
Vector3<double> BoundingVolumeHierarchy<MeshType>::ComputeCentroid(
    const MeshType& mesh, const IndexType i) {
  Vector3<double> centroid{0, 0, 0};
  const auto& element = mesh.element(i);
  // Calculate average from all vertices.
  for (int v = 0; v < kElementVertexCount; ++v) {
    const auto& vertex = mesh.vertex(element.vertex(v)).r_MV();
    centroid += vertex;
  }
  centroid /= kElementVertexCount;
  return centroid;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake

template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::SurfaceMesh<double>>;
template class drake::geometry::internal::BoundingVolumeHierarchy<
    drake::geometry::VolumeMesh<double>>;
