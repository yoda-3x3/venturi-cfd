#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/SparseLU>

namespace cfd::solvers {

// Rectangular obstacle mounted on the bottom wall, in physical units --
// port of solver/navier_stokes.py's Obstacle.
struct Obstacle2D {
    double x0 = 0.0, width = 0.0, height = 0.0;
    [[nodiscard]] double x1() const { return x0 + width; }
};

enum class ScenarioKind2D { Cavity, Channel, ObstacleScenario, ImageObstacle };

// Port of solver/navier_stokes.py's SolverConfig.
struct SolverConfig2D {
    int nx = 81, ny = 81;
    double Lx = 1.0, Ly = 1.0;
    double Re = 100.0;
    ScenarioKind2D kind = ScenarioKind2D::Cavity;
    double U = 1.0; // lid speed (cavity) or inflow speed (channel/obstacle/image)
    std::optional<Obstacle2D> obstacle;
    // Only used when kind == ImageObstacle: an arbitrary solid mask, row-
    // major (ny,nx) via core::idx2 (same layout as Fields2D), 1 = solid.
    // Must be exactly nx*ny elements. Takes the place of the rectangular
    // `obstacle` above -- the channel-flow boundary treatment (inflow at
    // the left column, outflow at the right, no-slip top/bottom) is
    // otherwise identical to ObstacleScenario, just with an arbitrary
    // shape instead of a rectangle.
    std::optional<std::vector<std::uint8_t>> custom_solid_mask;
    std::optional<double> dt; // nullopt => auto-computed stable dt
    double safety_factor = 0.5;
};

// Interior fields, matching solver/navier_stokes.py's fields() dict --
// row-major (ny, nx), i.e. flat[j*nx+i], via core::idx2.
struct Fields2D {
    std::vector<double> velocity_u, velocity_v, velocity_magnitude, vorticity, streamfunction;
    std::vector<float> obstacle;
};

// 2D incompressible Navier-Stokes solver, vorticity-streamfunction
// formulation on a collocated grid -- port of solver/navier_stokes.py's
// VorticityStreamSolver. The streamfunction Laplacian is assembled and
// LU-factorized once at construction (Eigen::SparseLU replacing
// scipy.sparse.csc_matrix + splu) and reused every step, since the matrix
// depends only on grid/geometry, not the solution.
class NavierStokes2D {
public:
    explicit NavierStokes2D(SolverConfig2D config);

    double step(); // advances one time step, returns the psi-update residual

    [[nodiscard]] Fields2D fields() const;

    [[nodiscard]] double time() const { return time_; }
    [[nodiscard]] int step_count() const { return step_count_; }
    [[nodiscard]] double last_residual() const { return last_residual_; }
    [[nodiscard]] double dt() const { return dt_; }
    [[nodiscard]] double dx() const { return dx_; }
    [[nodiscard]] double dy() const { return dy_; }

private:
    void classify_nodes();
    void set_fixed_psi_values();
    void build_and_factorize_laplacian();
    [[nodiscard]] double auto_dt() const;
    struct WallDir { int dj, di; double ut, h; };
    [[nodiscard]] double wall_omega(int j, int i, const std::vector<WallDir>& dirs) const;
    void update_boundary_and_obstacle_vorticity();
    void compute_velocity();
    [[nodiscard]] std::vector<double> vorticity_transport() const;
    double solve_psi();

    SolverConfig2D cfg_;
    int nx_, ny_;
    double dx_, dy_;
    std::vector<double> x_, y_;

    std::vector<double> psi_, omega_, u_, v_; // row-major (ny,nx)
    std::vector<std::uint8_t> solid_;
    std::vector<std::uint8_t> is_left_, is_right_, is_bottom_, is_top_;
    std::vector<std::uint8_t> fixed_, unknown_;
    std::vector<int> node_index_; // -1 or index into the unknown-node linear system
    int n_unknown_ = 0;
    bool is_channel_ = false;
    double Q_ = 0.0;

    Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_;

    double time_ = 0.0;
    int step_count_ = 0;
    double last_residual_ = 0.0;
    double dt_;
};

} // namespace cfd::solvers
