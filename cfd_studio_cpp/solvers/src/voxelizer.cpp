#include "solvers/voxelizer.hpp"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>

#include "core/connected_components.hpp"
#include "mesh/cap_boundaries.hpp"
#include "mesh/mesh_bvh.hpp"

namespace cfd::solvers {

using cfd::mesh::Mesh;
using cfd::mesh::MeshBVH;
using cfd::mesh::Vec3;

namespace {
inline std::size_t idx3d(int i, int j, int k, int ny, int nz) {
    return static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
         + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz)
         + static_cast<std::size_t>(k);
}

// One point-in-mesh sample per cell (at `subdiv`=1, its center) marks a
// whole cell solid/fluid -- fine for most geometry, but a thin feature
// (an aircraft wing, a fin) can be thinner than one coarse grid cell, and
// at low grid resolution it's entirely possible for the mesh to pass
// through many cells' *volumes* while missing every single cell *center*,
// yielding a solid mask that's empty everywhere even though the mesh
// clearly intersects the grid. `subdiv`>1 samples an NxNxN sub-grid
// within each cell instead and marks it solid if any sub-sample hits,
// closing that gap at the cost of subdiv^3 BVH queries per cell.
std::vector<std::uint8_t> sample_mask(const MeshBVH& bvh, int nx, int ny, int nz, double Lx, double Ly, double Lz,
                                       int subdiv) {
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz), 0);
    double dx = Lx / nx, dy = Ly / ny, dz = Lz / nz;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                bool solid = false;
                for (int su = 0; su < subdiv && !solid; ++su) {
                    double x = i * dx + (su + 0.5) / subdiv * dx;
                    for (int sv = 0; sv < subdiv && !solid; ++sv) {
                        double y = j * dy + (sv + 0.5) / subdiv * dy;
                        for (int sw = 0; sw < subdiv && !solid; ++sw) {
                            double z = k * dz + (sw + 0.5) / subdiv * dz;
                            solid = bvh.contains(Vec3{x, y, z});
                        }
                    }
                }
                mask[idx3d(i, j, k, ny, nz)] = solid ? 1 : 0;
            }
        }
    }
    return mask;
}

bool any_set(const std::vector<std::uint8_t>& mask) {
    for (auto v : mask) {
        if (v) return true;
    }
    return false;
}
} // namespace

PreparedGeometry prepare_geometry(
    const Mesh& mesh_in, double target_max_extent, double inflow_gap, double wake_gap, double lateral_gap,
    bool ground_effect, double altitude_gap) {
    Mesh mesh = mesh_in;
    auto b0 = mesh.bounds();
    Vec3 original_extents = b0.extents();

    Vec3 centroid = b0.center();
    mesh.translate(centroid * -1.0);

    double max_extent0 = std::max({original_extents.x, original_extents.y, original_extents.z});
    double scale = target_max_extent / max_extent0;
    mesh.scale(scale);

    auto b1 = mesh.bounds();
    Vec3 ext = b1.extents();
    double L = std::max({ext.x, ext.y, ext.z});
    double Lx = inflow_gap * L + ext.x + wake_gap * L;
    // Ground effect: decouple the bottom (y=0, "ground") clearance from the
    // top's, instead of centering symmetrically -- top clearance still
    // uses lateral_gap, same as the unmodified case.
    double Ly = ground_effect ? (ext.y + altitude_gap * L + lateral_gap * L) : (ext.y + 2 * lateral_gap * L);
    double Lz = ext.z + 2 * lateral_gap * L;

    Vec3 shift{
        inflow_gap * L - b1.min.x,
        ground_effect ? (altitude_gap * L - b1.min.y) : (Ly / 2 - (b1.min.y + b1.max.y) / 2),
        Lz / 2 - (b1.min.z + b1.max.z) / 2,
    };
    mesh.translate(shift);

    PreparedGeometry result;
    result.is_watertight = cfd::mesh::is_watertight(mesh);
    result.mesh = std::move(mesh);
    result.Lx = Lx;
    result.Ly = Ly;
    result.Lz = Lz;
    result.original_extents = original_extents;
    return result;
}

PreparedGeometry prepare_internal_geometry(
    const Mesh& mesh_in, double target_max_extent, double inflow_gap, double wake_gap) {
    Mesh mesh = mesh_in;
    auto b0 = mesh.bounds();
    Vec3 original_extents = b0.extents();

    Vec3 centroid = b0.center();
    mesh.translate(centroid * -1.0);

    double max_extent0 = std::max({original_extents.x, original_extents.y, original_extents.z});
    double scale = target_max_extent / max_extent0;
    mesh.scale(scale);

    auto b1 = mesh.bounds();
    Vec3 ext = b1.extents();
    double L = std::max({ext.x, ext.y, ext.z});
    double Lx = ext.x + (inflow_gap + wake_gap) * L;
    double Ly = ext.y;
    double Lz = ext.z;

    Vec3 shift{
        inflow_gap * L - b1.min.x,
        Ly / 2 - (b1.min.y + b1.max.y) / 2,
        Lz / 2 - (b1.min.z + b1.max.z) / 2,
    };
    mesh.translate(shift);

    PreparedGeometry result;
    result.is_watertight = cfd::mesh::is_watertight(mesh);
    result.mesh = std::move(mesh);
    result.Lx = Lx;
    result.Ly = Ly;
    result.Lz = Lz;
    result.original_extents = original_extents;
    return result;
}

std::vector<std::uint8_t> voxelize_to_grid(const Mesh& mesh, int nx, int ny, int nz, double Lx, double Ly, double Lz) {
    MeshBVH bvh(mesh);
    // Single center sample per cell is the fast, common-case path (matches
    // this function's cost before this fix, unchanged for every geometry
    // that doesn't hit the thin-feature gap sample_mask()'s doc comment
    // describes). Only pay for the denser retry on the geometries that
    // actually need it: an empty result at nx*ny*nz cells almost certainly
    // means at least one dimension of the object is thinner than a grid
    // cell somewhere, not that the object is genuinely absent from the
    // domain (prepare_geometry() always places it well inside the grid).
    std::vector<std::uint8_t> mask = sample_mask(bvh, nx, ny, nz, Lx, Ly, Lz, 1);
    if (!any_set(mask)) mask = sample_mask(bvh, nx, ny, nz, Lx, Ly, Lz, 4);
    return mask;
}

std::vector<std::uint8_t> voxelize_internal_to_grid(
    const Mesh& mesh_in, int nx, int ny, int nz, double Lx, double Ly, double Lz) {
    Mesh mesh = cfd::mesh::is_watertight(mesh_in) ? mesh_in : cfd::mesh::cap_open_boundaries(mesh_in);

    MeshBVH bvh(mesh);
    std::size_t n = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
    // Same thin-wall-vs-coarse-grid gap as voxelize_to_grid: a thin pipe
    // wall can fall entirely between single-center-sample cells, wrongly
    // reading as "no wall material anywhere" -- retry denser before
    // concluding that.
    std::vector<std::uint8_t> inside_mask = sample_mask(bvh, nx, ny, nz, Lx, Ly, Lz, 1);
    if (!any_set(inside_mask)) inside_mask = sample_mask(bvh, nx, ny, nz, Lx, Ly, Lz, 4);

    std::vector<std::uint8_t> complement(n);
    for (std::size_t i = 0; i < n; ++i) complement[i] = inside_mask[i] ? 0 : 1;

    core::ComponentLabels labeled = core::label_components_6connectivity(complement, nx, ny, nz);

    // Only the lateral (y,z) domain faces count as "true exterior" -- the
    // pipe is open at its two x-axis ends by design (inlet/outlet), so an
    // enclosed lumen legitimately reaches x=0/x=Lx and must not be
    // disqualified for touching them.
    std::unordered_set<std::int32_t> boundary_labels;
    for (int i = 0; i < nx; ++i) {
        for (int k = 0; k < nz; ++k) {
            boundary_labels.insert(labeled.labels[idx3d(i, 0, k, ny, nz)]);
            boundary_labels.insert(labeled.labels[idx3d(i, ny - 1, k, ny, nz)]);
        }
        for (int j = 0; j < ny; ++j) {
            boundary_labels.insert(labeled.labels[idx3d(i, j, 0, ny, nz)]);
            boundary_labels.insert(labeled.labels[idx3d(i, j, nz - 1, ny, nz)]);
        }
    }
    boundary_labels.erase(0); // label 0 is inside_mask itself, not a complement component

    std::vector<std::uint8_t> fluid_mask(n, 0);
    bool any_fluid = false;
    for (std::size_t i = 0; i < n; ++i) {
        bool is_fluid = complement[i] != 0 && !boundary_labels.count(labeled.labels[i]);
        fluid_mask[i] = is_fluid ? 1 : 0;
        any_fluid = any_fluid || is_fluid;
    }

    if (!any_fluid) {
        // No enclosed wall-shell cavity: the mesh directly bounds the lumen.
        fluid_mask = inside_mask;
        any_fluid = std::any_of(inside_mask.begin(), inside_mask.end(), [](std::uint8_t v) { return v != 0; });
    }

    if (!any_fluid) {
        throw std::runtime_error(
            "No enclosed interior volume (lumen) found in the uploaded geometry -- "
            "check that the file represents a hollow duct/pipe.");
    }

    std::vector<std::uint8_t> solid_mask(n);
    for (std::size_t i = 0; i < n; ++i) solid_mask[i] = fluid_mask[i] ? 0 : 1;
    return solid_mask;
}

} // namespace cfd::solvers
