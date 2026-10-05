#pragma once

#include <cstdint>
#include <vector>

#include "solvers/navier_stokes_2d.hpp"

namespace cfd::solvers {

// Which edge of the user's uploaded image the flow enters from. The solver
// itself (NavierStokes2D) only ever knows how to run "left column inflow,
// right column outflow, top/bottom no-slip walls" -- ImageObstacle reuses
// that unmodified, well-tested boundary treatment for every direction by
// reorienting the mask before construction (orient_mask_for_solver) and
// reorienting the results back after (orient_fields_for_display), rather
// than teaching the solver itself about arbitrary inflow edges.
enum class ImageFlowDirection { Right, Left, Up, Down };

struct OrientedMask2D {
    std::vector<std::uint8_t> mask; // row-major (ny,nx) via idx2, ready for SolverConfig2D::custom_solid_mask
    int nx = 0, ny = 0;
};

// `display_mask` uses the same (ny,nx) row-major layout as Fields2D/idx2:
// row 0 is the bottom of the image (physical y=0), column 0 is the left
// (physical x=0) -- i.e. it's already how the image will look once drawn
// by PlotWidget (which itself flips field row 0 to the bottom of the
// screen). See orient_fields_for_display() for the exact per-direction
// transform this inverts.
[[nodiscard]] OrientedMask2D orient_mask_for_solver(const std::vector<std::uint8_t>& display_mask, int display_nx,
                                                     int display_ny, ImageFlowDirection dir);

// Inverse of orient_mask_for_solver, applied to a solved Fields2D: maps the
// solver's own always-left-to-right fields back into the orientation the
// user's uploaded image was in, so the on-screen result (and the written
// VTK series) shows flow moving the direction they actually drew, through
// their image right-side up.
//
// Right is the solver's native orientation, so it's a pure passthrough.
// Left is a horizontal mirror (x -> -x): an IMPROPER transform (determinant
// -1), which negates the pseudo-scalars vorticity and streamfunction (and
// the u component) the same way a mirror reverses handedness. Up is a
// coordinate transpose (also improper, another axis reflection) --
// likewise negates vorticity/streamfunction, with u/v swapped rather than
// negated since it's the axes being swapped, not one of them flipped. Down
// is transpose+flip, which composes to a PROPER 90 degree rotation
// (determinant +1) -- vorticity/streamfunction are unchanged, only the
// velocity components swap (with a sign flip on the new v). Each of these
// was independently re-derived from u=d(psi)/dy, v=-d(psi)/dx and cross-
// checked for self-consistency (see the commit this was introduced in),
// not just asserted -- this is exactly the class of sign mistake that a
// vorticity solver is easy to get quietly wrong in.
[[nodiscard]] Fields2D orient_fields_for_display(const Fields2D& solver_fields, int solver_nx, int solver_ny,
                                                  ImageFlowDirection dir, int& display_nx, int& display_ny);

} // namespace cfd::solvers
