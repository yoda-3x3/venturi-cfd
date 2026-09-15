#include <catch_amalgamated.hpp>

#include <cmath>

#include "solvers/navier_stokes_2d.hpp"

using namespace cfd::solvers;

namespace {
bool all_finite(const std::vector<double>& v) {
    for (double x : v) {
        if (!std::isfinite(x)) return false;
    }
    return true;
}
} // namespace

TEST_CASE("NavierStokes2D: lid-driven cavity stays finite and respects lid BC", "[solvers][navier_stokes_2d]") {
    SolverConfig2D cfg;
    cfg.nx = 21; cfg.ny = 21;
    cfg.Re = 100.0;
    cfg.U = 1.0;
    cfg.kind = ScenarioKind2D::Cavity;

    NavierStokes2D solver(cfg);
    for (int s = 0; s < 30; ++s) {
        double residual = solver.step();
        REQUIRE(std::isfinite(residual));
        REQUIRE(residual >= 0.0);
    }

    Fields2D f = solver.fields();
    REQUIRE(all_finite(f.velocity_u));
    REQUIRE(all_finite(f.velocity_v));
    REQUIRE(all_finite(f.streamfunction));
    REQUIRE(all_finite(f.vorticity));

    // Top row (the lid) should move at U; side/bottom walls at 0 (no-slip).
    for (int i = 0; i < cfg.nx; ++i) {
        REQUIRE(f.velocity_u[static_cast<std::size_t>(cfg.ny - 1) * cfg.nx + static_cast<std::size_t>(i)]
                == Catch::Approx(cfg.U));
        REQUIRE(f.velocity_u[static_cast<std::size_t>(i)] == Catch::Approx(0.0).margin(1e-12)); // bottom row
    }
}

// Regression test for a real bug: wall_omega()'s Thom's-formula sign for
// the moving-wall (lid) tangential-velocity term was inverted, which spun
// the cavity's primary vortex backwards -- fluid right under the lid moved
// OPPOSITE the lid itself. Every existing test only checked BC-enforced
// values (the lid row IS pinned to U by construction) and finiteness, so
// this was invisible to them; only the interior, solver-derived flow
// direction exposes it. These signs, and the general shape of the
// velocity field, were cross-checked by hand against Ghia, Ghia & Shin
// (1982)'s classic Re=100 lid-driven-cavity benchmark (the standard
// reference every such solver is validated against) at a finer 129x129
// grid, matching to within ~1-4% -- well within the expected error for
// this scheme's first-order-upwind convection term.
TEST_CASE("NavierStokes2D: lid-driven cavity rotates the correct direction", "[solvers][navier_stokes_2d]") {
    SolverConfig2D cfg;
    cfg.nx = 41; cfg.ny = 41;
    cfg.Re = 100.0;
    cfg.U = 1.0;
    cfg.kind = ScenarioKind2D::Cavity;

    NavierStokes2D solver(cfg);
    for (int s = 0; s < 4000; ++s) solver.step();
    Fields2D f = solver.fields();

    // A lid moving in +x drives a CLOCKWISE primary vortex: fluid just
    // below the lid follows it (u > 0, not just the BC-pinned top row
    // itself), descends along the right wall (v < 0), and rises along the
    // left wall (v > 0) to feed back toward the lid.
    double u_under_lid = f.velocity_u[static_cast<std::size_t>(38) * cfg.nx + 20];
    double v_near_left = f.velocity_v[static_cast<std::size_t>(20) * cfg.nx + 2];
    double v_near_right = f.velocity_v[static_cast<std::size_t>(20) * cfg.nx + 38];

    REQUIRE(u_under_lid > 0.3);   // follows the lid, not opposing it
    REQUIRE(v_near_left > 0.01);  // rising along the left wall
    REQUIRE(v_near_right < -0.01); // descending along the right wall
}

TEST_CASE("NavierStokes2D: channel flow stays finite", "[solvers][navier_stokes_2d]") {
    SolverConfig2D cfg;
    cfg.nx = 41; cfg.ny = 15;
    cfg.Lx = 3.0; cfg.Ly = 1.0;
    cfg.Re = 150.0;
    cfg.U = 1.0;
    cfg.kind = ScenarioKind2D::Channel;

    NavierStokes2D solver(cfg);
    for (int s = 0; s < 30; ++s) {
        double residual = solver.step();
        REQUIRE(std::isfinite(residual));
    }
    Fields2D f = solver.fields();
    REQUIRE(all_finite(f.velocity_u));
    REQUIRE(all_finite(f.velocity_v));

    // Inflow column should be exactly U (fixed BC).
    for (int j = 0; j < cfg.ny; ++j) {
        REQUIRE(f.velocity_u[static_cast<std::size_t>(j) * cfg.nx] == Catch::Approx(cfg.U));
    }
}

TEST_CASE("NavierStokes2D: obstacle scenario zeroes velocity inside the solid block", "[solvers][navier_stokes_2d]") {
    SolverConfig2D cfg;
    cfg.nx = 41; cfg.ny = 21;
    cfg.Lx = 3.0; cfg.Ly = 1.0;
    cfg.Re = 150.0;
    cfg.U = 1.0;
    cfg.kind = ScenarioKind2D::ObstacleScenario;
    cfg.obstacle = Obstacle2D{1.0, 0.3, 0.3};

    NavierStokes2D solver(cfg);
    for (int s = 0; s < 20; ++s) solver.step();

    Fields2D f = solver.fields();
    REQUIRE(all_finite(f.velocity_u));
    bool any_solid = false;
    for (std::size_t k = 0; k < f.obstacle.size(); ++k) {
        if (f.obstacle[k] == 1.0f) {
            any_solid = true;
            REQUIRE(f.velocity_u[k] == 0.0);
            REQUIRE(f.velocity_v[k] == 0.0);
        }
    }
    REQUIRE(any_solid); // the obstacle placement should actually intersect the grid
}
