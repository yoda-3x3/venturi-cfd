#include <catch_amalgamated.hpp>

#include "mesh/mesh.hpp"
#include "mesh/mesh_bvh.hpp"

using namespace cfd::mesh;

namespace {
// Unit-right-angle tetrahedron: (0,0,0),(1,0,0),(0,1,0),(0,0,1).
Mesh make_tetrahedron() {
    Mesh mesh;
    mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    mesh.triangles = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    return mesh;
}
} // namespace

TEST_CASE("MeshBVH::contains: centroid of a closed mesh is inside", "[mesh]") {
    MeshBVH bvh(make_tetrahedron());
    REQUIRE(bvh.contains(Vec3{0.2, 0.2, 0.2}));
}

TEST_CASE("MeshBVH::contains: a point far outside the mesh is outside", "[mesh]") {
    MeshBVH bvh(make_tetrahedron());
    REQUIRE_FALSE(bvh.contains(Vec3{10.0, 10.0, 10.0}));
    REQUIRE_FALSE(bvh.contains(Vec3{-1.0, -1.0, -1.0}));
}

// Regression test for a real bug: containment used to be decided by a
// SINGLE fixed-direction ray's crossing parity. A lone stray triangle,
// floating in space with no relation to any actual solid, sitting on that
// one ray's path from the query point, was enough to flip the result --
// exactly the failure mode that turned a non-watertight upload
// (avion31.stl) into a spurious solid mask reaching all the way to the
// domain floor, far from the actual object (diagnosed via cfd_headless +
// dumping a frame's obstacle field; see this fix's commit).
//
// This constructs that exact scenario in miniature: a single triangle
// placed directly on ray direction 0's path from the origin (so ray 0
// sees an odd/"inside" crossing), positioned far enough off ray
// directions 1 and 2's paths (their X and Y coordinates stay 0 along
// their whole length, but the triangle sits at X~2.6/Y~4.3) that neither
// of those ever reaches it (0 crossings/"outside" each). A single-ray
// test using direction 0 alone would wrongly call the origin "inside";
// the majority vote (1 of 3) correctly calls it outside.
TEST_CASE("MeshBVH::contains: one ray hit by a stray triangle is outvoted by two that miss it",
          "[mesh]") {
    Mesh mesh;
    // Centered exactly on ray direction 0 ==
    // (0.5257311121, 0.8506508084, 0)'s path at distance 5 from the
    // origin, built from two vectors perpendicular to it (e1 = (0,0,1);
    // e2 = dir0 x e1) so the triangle's centroid -- and therefore a point
    // strictly inside it -- lands exactly on the ray. Ray directions 1
    // and 2's own points never leave X=0 / Y=0 respectively, so neither
    // comes anywhere near this triangle regardless of its size.
    Vec3 center{5.0 * 0.5257311121, 5.0 * 0.8506508084, 0.0};
    Vec3 e1{0.0, 0.0, 1.0};
    Vec3 e2{0.8506508084, -0.5257311121, 0.0};
    mesh.vertices = {
        center + e2 * 0.3,
        center + e1 * 0.26 - e2 * 0.15,
        center - e1 * 0.26 - e2 * 0.15,
    };
    mesh.triangles = {{0, 1, 2}};

    MeshBVH bvh(mesh);
    REQUIRE_FALSE(bvh.contains(Vec3{0.0, 0.0, 0.0}));
}

TEST_CASE("MeshBVH::nearest_hit: ray straight into a known face returns the expected point", "[mesh]") {
    MeshBVH bvh(make_tetrahedron());
    // A ray from above straight down at (0.2, 0.2, *) first crosses the
    // slanted face (1,2,3) -- plane x+y+z=1 -- at z = 1-0.2-0.2 = 0.6,
    // which is nearer to the ray origin than the z=0 base face behind it;
    // rtcIntersect1 returns only the nearest hit.
    auto hit = bvh.nearest_hit(Vec3{0.2, 0.2, 5.0}, Vec3{0.0, 0.0, -1.0});
    REQUIRE(hit.has_value());
    REQUIRE(hit->point.x == Catch::Approx(0.2).margin(1e-4));
    REQUIRE(hit->point.y == Catch::Approx(0.2).margin(1e-4));
    REQUIRE(hit->point.z == Catch::Approx(0.6).margin(1e-4));
}

TEST_CASE("MeshBVH::nearest_hit: a ray that misses the mesh entirely returns nullopt", "[mesh]") {
    MeshBVH bvh(make_tetrahedron());
    auto hit = bvh.nearest_hit(Vec3{10.0, 10.0, 10.0}, Vec3{1.0, 0.0, 0.0});
    REQUIRE_FALSE(hit.has_value());
}
