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
    // int axis{};
    // aabb.half_width().maxCoeff(&axis);

    // double optimal_axis = axis;
    // std::sort(start, end,
    //           [optimal_axis](const CentroidPair& a, const CentroidPair& b) {
    //             return a.second[optimal_axis] < b.second[optimal_axis];
    //           });

    typename std::vector<CentroidPair>::iterator optimal_split;
    if (h) {
      int optimal_axis = 0;

      double optimal_volume = std::numeric_limits<double>::max();
      for (int axis = 0; axis < 3; ++axis) {
        std::sort(start, end,
                  [axis, mesh](const CentroidPair& a, const CentroidPair& b) {
                    return a.second[axis] < b.second[axis];
                    // const auto& element_a = mesh.element(a.first);
                    // const auto& element_b= mesh.element(b.first);
                    // Vector3<double> max_a, min_a, max_b, min_b;
                    // max_a.setConstant(std::numeric_limits<double>::lowest());
                    // min_a.setConstant(std::numeric_limits<double>::max());
                    // max_b.setConstant(std::numeric_limits<double>::lowest());
                    // min_b.setConstant(std::numeric_limits<double>::max());
                    // // Check each vertex in the element.
                    // for (int v = 0; v < kElementVertexCount; ++v) {
                    //   const auto& vertex_a =
                    //   mesh.vertex(element_a.vertex(v)).r_MV();
                    //   // Compare its extent along each of the 3 axes.
                    //   min_a = min_a.cwiseMin(vertex_a);
                    //   max_a = max_a.cwiseMax(vertex_a);
                    //   const auto& vertex_b =
                    //   mesh.vertex(element_b.vertex(v)).r_MV();
                    //   // Compare its extent along each of the 3 axes.
                    //   min_b = min_b.cwiseMin(vertex_b);
                    //   max_b = max_b.cwiseMax(vertex_b);
                    // }
                    // Vector3<double> center_a = (min_a + max_a) / 2;
                    // Vector3<double> center_b = (min_b + max_b) / 2;
                    // if (max_a[axis] < min_b[axis]) {
                    //   return true;
                    // } else if (min_a[axis] > max_b[axis]) {
                    //   return false;
                    // } else if (max_a[axis] < max_b[axis]) {
                    //   return true;
                    // } else if (max_a[axis] > max_b[axis]) {
                    //   return false;
                    // } else if (center_a[(axis + 1) % 3] < center_b[(axis + 1)
                    // % 3]) {
                    //   return true;
                    // } else if (center_a[(axis + 1) % 3] > center_b[(axis + 1)
                    // % 3]) {
                    //   return false;
                    // } else {
                    //   return center_a[(axis + 2) % 3] < center_b[(axis + 2) %
                    //   3];
                    // }
                  });

        if (num_elements == 512) {
          std::cout << "Axis " << axis << std::endl;
          int num_before = 0;
          int num_both = 0;
          int num_after = 0;
          for (typename std::vector<CentroidPair>::iterator cur = start;
               cur < end; ++cur) {
            auto box = ComputeBoundingVolume(mesh, cur, cur + 1);
            if (box.upper()[axis] < -1) {
              ++num_before;
            } else if (box.lower()[axis] > -1) {
              ++num_after;
            } else {
              ++num_both;
            }
          }
          std::cout << " Elements along axis " << num_before << " " << num_both
                    << " " << num_after << std::endl;
        }

        for (typename std::vector<CentroidPair>::iterator split = start + 1;
            split < end; ++split) {
          auto a = ComputeBoundingVolume(mesh, start, split);
          auto b = ComputeBoundingVolume(mesh, split, end);
          // Calculate overlapping volume.
          double x = std::max(std::min(a.upper().x(), b.upper().x()) - std::max(a.lower().x(), b.lower().x()), 0.);
          double y = std::max(std::min(a.upper().y(), b.upper().y()) -
                                  std::max(a.lower().y(), b.lower().y()),
                              0.);
          double z = std::max(std::min(a.upper().z(), b.upper().z()) -
                                  std::max(a.lower().z(), b.lower().z()),
                              0.);
          double overlap_volume = x * y * z;

          const double child_volume =
              ComputeBoundingVolume(mesh, start, split).CalcVolume() +
              ComputeBoundingVolume(mesh, split, end).CalcVolume() + overlap_volume;


          if (axis == 0 && num_elements == 512 &&
              ((split - start) == 1 || (split - start) == 55 ||
               (split - start) == 56 || (split - start) == 57 ||
               (split - start) == 143 || (split - start) == 144 ||
               (split - start) == 145)) {
            std::cout << std::endl;
            std::cout << "Comparing volumes at " << (split - start)
                      << std::endl;
            std::cout << "L : " << a.lower().transpose()
                      << " U: " << a.upper().transpose() << " to "
                      << "L: " << b.lower().transpose()
                      << " U: " << b.upper().transpose() << std::endl;
            std::cout << "A vol: " << a.CalcVolume()
                      << " + B vol: " << b.CalcVolume() << " = " << child_volume
                      << ". Best:" << optimal_volume << " at "
                       << (optimal_split - start) << std::endl;
                      std::cout << "Overlap volume " << overlap_volume << std::endl;


            const auto& ele = mesh.element(split->first);
            Vector3<double> max_bounds, min_bounds;
            max_bounds.setConstant(std::numeric_limits<double>::lowest());
            min_bounds.setConstant(std::numeric_limits<double>::max());

            for (int v = 0; v < 3; ++v) {
              const auto& vertex = mesh.vertex(ele.vertex(v)).r_MV();
              std::cout << "  Vertices " << vertex.transpose() << std::endl;
              min_bounds = min_bounds.cwiseMin(vertex);
              max_bounds = max_bounds.cwiseMax(vertex);
            }
            std::cout << "Centroid " << split->second.transpose() << std::endl;
            auto center = (min_bounds + max_bounds) / 2;
            auto half_width = max_bounds - center;
            std::cout << "Min " << min_bounds.transpose() << " max "
                      << max_bounds.transpose() << std::endl;
            std::cout << "Center " << center.transpose()
                      << " Halfwidth " << half_width.transpose() << std::endl;
          }

          if (child_volume < optimal_volume) {
            optimal_axis = axis;
            optimal_volume = child_volume;
            optimal_split = split;
            if (num_elements == 512) std::cout << "Updating split to " << (optimal_split - start) << std::endl;
          }
        }
      }
      // std::cout << "Optimal volume " << optimal_volume << " over num elements "
      //           << num_elements << " split at " << (optimal_split - start) << " along axis " << optimal_axis << std::endl;
      // if (optimal_axis != 2) {
      //   std::sort(
      //       start, end, [optimal_axis, mesh](const CentroidPair& a, const CentroidPair& b) {
      //         const auto& element_a = mesh.element(a.first);
      //         const auto& element_b = mesh.element(b.first);
      //         Vector3<double> max_a, min_a, max_b, min_b;
      //         max_a.setConstant(std::numeric_limits<double>::lowest());
      //         min_a.setConstant(std::numeric_limits<double>::max());
      //         max_b.setConstant(std::numeric_limits<double>::lowest());
      //         min_b.setConstant(std::numeric_limits<double>::max());
      //         // Check each vertex in the element.
      //         for (int v = 0; v < kElementVertexCount; ++v) {
      //           const auto& vertex_a = mesh.vertex(element_a.vertex(v)).r_MV();
      //           // Compare its extent along each of the 3 axes.
      //           min_a = min_a.cwiseMin(vertex_a);
      //           max_a = max_a.cwiseMax(vertex_a);
      //           const auto& vertex_b = mesh.vertex(element_b.vertex(v)).r_MV();
      //           // Compare its extent along each of the 3 axes.
      //           min_b = min_b.cwiseMin(vertex_b);
      //           max_b = max_b.cwiseMax(vertex_b);
      //         }
      //         Vector3<double> center_a = (min_a + max_a) / 2;
      //         Vector3<double> center_b = (min_b + max_b) / 2;
      //         if (max_a[optimal_axis] < min_b[optimal_axis]) {
      //           return true;
      //         } else if (min_a[optimal_axis] > max_b[optimal_axis]) {
      //           return false;
      //         } else if (max_a[optimal_axis] < max_b[optimal_axis]) {
      //           return true;
      //         } else if (max_a[optimal_axis] > max_b[optimal_axis]) {
      //           return false;
      //         } else if (center_a[(optimal_axis + 1) % 3] < center_b[(optimal_axis + 1) % 3]) {
      //           return true;
      //         } else if (center_a[(optimal_axis + 1) % 3] > center_b[(optimal_axis + 1) % 3]) {
      //           return false;
      //         } else {
      //           return center_a[(optimal_axis + 2) % 3] < center_b[(optimal_axis + 2) % 3];
      //         }
      //       });
      // }
      // std::cout << "Optimal split is " << (optimal_split - start)  <<
      // " out of " << num_elements << " elements" << std::endl;
    } else {
      int axis{};
      aabb.half_width().maxCoeff(&axis);
      double optimal_axis = axis;
      std::sort(start, end,
              [optimal_axis](const CentroidPair& a, const CentroidPair& b) {
                return a.second[optimal_axis] < b.second[optimal_axis];
              });
      optimal_split = start + num_elements / 2;
      const double test_volume =
          ComputeBoundingVolume(mesh, start, optimal_split).CalcVolume() +
          ComputeBoundingVolume(mesh, optimal_split, end).CalcVolume();
      std::cout << "Volume " << test_volume << " over num elements "
                << num_elements << std::endl;
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
