#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "mesh/mesh.hpp"

namespace cfd::mesh {

struct RayHit {
    Vec3 point;
    double distance = 0.0;
    std::uint32_t triangle_index = 0;
};

// Embree-backed BVH over a triangle mesh, shared by both mesh-containment
// voxelization (solver3d/geometry.py's mesh.contains(points), ported) and
// interactive picking in the orientation dialog -- one BVH, two use sites,
// eliminating the Python app's separate face-decimation-for-preview and
// occlusion-disambiguation-by-depth-sort machinery (both existed only
// because mplot3d has no hardware acceleration / real ray casting).
//
// Not copyable (owns Embree device/scene handles); movable.
class MeshBVH {
public:
    explicit MeshBVH(const Mesh& mesh);
    ~MeshBVH();

    MeshBVH(const MeshBVH&) = delete;
    MeshBVH& operator=(const MeshBVH&) = delete;
    MeshBVH(MeshBVH&&) noexcept;
    MeshBVH& operator=(MeshBVH&&) noexcept;

    // Point-in-mesh containment test via ray-casting parity: casts 3 rays
    // (different, non-axis-aligned directions) from `point` and counts
    // triangle crossings along each; a ray with an odd crossing count votes
    // "inside", and the majority (2 of 3) wins. A single ray is fragile
    // against a non-watertight mesh -- one hole anywhere along that one
    // direction flips every point behind it, however far from the real
    // surface -- so 3 differently-angled rays are used and voted rather
    // than trusted individually, since a hole that misdirects one of them
    // is common but one that misdirects a majority is not.
    [[nodiscard]] bool contains(const Vec3& point) const;

    // Nearest ray-mesh intersection, for interactive picking -- replaces
    // ui/orientation_dialog.py's proj3d.proj_transform depth-sort hack
    // entirely: Embree already returns the nearest hit along the ray.
    [[nodiscard]] std::optional<RayHit> nearest_hit(const Vec3& origin, const Vec3& direction) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cfd::mesh
