#include "solvers/navier_stokes_2d.hpp"

#include <algorithm>
#include <cmath>

#include "core/grid_index.hpp"

namespace cfd::solvers {

using core::idx2;

namespace {
// The 4 grid-neighbor offsets + Laplacian coefficient, in the exact order
// solver/navier_stokes.py's _build_and_factorize_laplacian/_solve_psi use
// (order doesn't affect correctness here since all 4 are always visited,
// but keeping it identical makes side-by-side comparison with the Python
// source easier).
struct NeighborOffset { int dj, di; };
} // namespace

NavierStokes2D::NavierStokes2D(SolverConfig2D config)
    : cfg_(config), nx_(config.nx), ny_(config.ny),
      dx_(config.Lx / (config.nx - 1)), dy_(config.Ly / (config.ny - 1)),
      dt_(0.0) {
    x_.resize(static_cast<std::size_t>(nx_));
    for (int i = 0; i < nx_; ++i) x_[static_cast<std::size_t>(i)] = config.Lx * i / (nx_ - 1);
    y_.resize(static_cast<std::size_t>(ny_));
    for (int j = 0; j < ny_; ++j) y_[static_cast<std::size_t>(j)] = config.Ly * j / (ny_ - 1);

    std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    psi_.assign(n, 0.0);
    omega_.assign(n, 0.0);
    u_.assign(n, 0.0);
    v_.assign(n, 0.0);
    solid_.assign(n, 0);

    if (cfg_.kind == ScenarioKind2D::ObstacleScenario && cfg_.obstacle) {
        const auto& ob = *cfg_.obstacle;
        for (int j = 0; j < ny_; ++j) {
            if (y_[static_cast<std::size_t>(j)] > ob.height) continue;
            for (int i = 0; i < nx_; ++i) {
                double xi = x_[static_cast<std::size_t>(i)];
                if (xi >= ob.x0 && xi <= ob.x1()) solid_[idx2(j, i, nx_)] = 1;
            }
        }
    }

    is_channel_ = cfg_.kind == ScenarioKind2D::Channel || cfg_.kind == ScenarioKind2D::ObstacleScenario;
    Q_ = cfg_.U * cfg_.Ly;

    classify_nodes();
    set_fixed_psi_values();
    build_and_factorize_laplacian();

    dt_ = cfg_.dt ? *cfg_.dt : auto_dt();
}

void NavierStokes2D::classify_nodes() {
    std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    std::vector<std::uint8_t> edge(n, 0);
    is_left_.assign(n, 0);
    is_right_.assign(n, 0);
    is_bottom_.assign(n, 0);
    is_top_.assign(n, 0);

    for (int j = 0; j < ny_; ++j) {
        edge[idx2(j, 0, nx_)] = 1;
        edge[idx2(j, nx_ - 1, nx_)] = 1;
        is_left_[idx2(j, 0, nx_)] = 1;
        is_right_[idx2(j, nx_ - 1, nx_)] = 1;
    }
    for (int i = 0; i < nx_; ++i) {
        edge[idx2(0, i, nx_)] = 1;
        edge[idx2(ny_ - 1, i, nx_)] = 1;
        is_bottom_[idx2(0, i, nx_)] = 1;
        is_top_[idx2(ny_ - 1, i, nx_)] = 1;
    }

    fixed_.assign(n, 0);
    unknown_.assign(n, 0);
    for (std::size_t k = 0; k < n; ++k) {
        fixed_[k] = (edge[k] || solid_[k]) ? 1 : 0;
        unknown_[k] = fixed_[k] ? 0 : 1;
    }

    node_index_.assign(n, -1);
    n_unknown_ = 0;
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            std::size_t k = idx2(j, i, nx_);
            if (unknown_[k]) node_index_[k] = n_unknown_++;
        }
    }
}

void NavierStokes2D::set_fixed_psi_values() {
    if (cfg_.kind == ScenarioKind2D::Cavity) {
        for (int j = 0; j < ny_; ++j)
            for (int i = 0; i < nx_; ++i)
                if (fixed_[idx2(j, i, nx_)]) psi_[idx2(j, i, nx_)] = 0.0;
        return;
    }

    // channel / obstacle: bottom wall + obstacle = 0, top wall = Q, inflow
    // column = linear profile U*y, outflow extrapolated later. Initialize
    // the WHOLE field (including interior) to the uniform inflow profile
    // so the initial condition is smooth/consistent instead of jumping
    // straight from 0 to Q over one grid cell.
    for (int j = 0; j < ny_; ++j) {
        double val = cfg_.U * y_[static_cast<std::size_t>(j)];
        for (int i = 0; i < nx_; ++i) psi_[idx2(j, i, nx_)] = val;
    }
    for (int j = 0; j < ny_; ++j)
        for (int i = 0; i < nx_; ++i)
            if (solid_[idx2(j, i, nx_)]) psi_[idx2(j, i, nx_)] = 0.0;
    for (int i = 0; i < nx_; ++i)
        if (is_top_[idx2(ny_ - 1, i, nx_)]) psi_[idx2(ny_ - 1, i, nx_)] = Q_;
    for (int j = 0; j < ny_; ++j) {
        double val = cfg_.U * y_[static_cast<std::size_t>(j)];
        psi_[idx2(j, 0, nx_)] = val;
        psi_[idx2(j, nx_ - 1, nx_)] = val;
    }
}

void NavierStokes2D::build_and_factorize_laplacian() {
    double dx2 = dx_ * dx_, dy2 = dy_ * dy_;
    double cx = 1.0 / dx2, cy = 1.0 / dy2;
    double cc = -2.0 * (cx + cy);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<std::size_t>(n_unknown_) * 5);

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            std::size_t k_flat = idx2(j, i, nx_);
            if (!unknown_[k_flat]) continue;
            int k = node_index_[k_flat];
            triplets.emplace_back(k, k, cc);

            static constexpr int offsets[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
            const double coeffs[4] = {cx, cx, cy, cy};
            for (int o = 0; o < 4; ++o) {
                int nj = j + offsets[o][0], ni = i + offsets[o][1];
                std::size_t nk_flat = idx2(nj, ni, nx_);
                if (unknown_[nk_flat]) {
                    triplets.emplace_back(k, node_index_[nk_flat], coeffs[o]);
                }
                // if neighbor is fixed, its contribution goes to the RHS at solve time
            }
        }
    }

    Eigen::SparseMatrix<double> A(n_unknown_, n_unknown_);
    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();
    lu_.compute(A);
}

double NavierStokes2D::auto_dt() const {
    // Combined explicit-scheme stability bound (diffusion + convection
    // acting together in the same forward-Euler update), rather than the
    // min of two separate limits, which underestimates the restriction
    // when both terms are significant at once.
    double Umax = std::max(cfg_.U, 1e-6);
    double diff_term = 2.0 * (1.0 / cfg_.Re) * (1.0 / (dx_ * dx_) + 1.0 / (dy_ * dy_));
    double conv_term = Umax / dx_ + Umax / dy_;
    double dt_max = 1.0 / (diff_term + conv_term);
    return cfg_.safety_factor * dt_max;
}

double NavierStokes2D::wall_omega(int j, int i, const std::vector<WallDir>& dirs) const {
    if (dirs.empty()) return 0.0;
    double psi_wall = psi_[idx2(j, i, nx_)];
    double sum = 0.0;
    for (const auto& d : dirs) {
        int nj = j + d.dj, ni = i + d.di;
        double psi_nb = psi_[idx2(nj, ni, nx_)];
        // Thom's formula for vorticity at a (possibly tangentially moving)
        // wall, derived from a Taylor expansion of psi about the wall
        // using the known wall-normal derivative (the tangential velocity
        // ut) and -omega ~= d^2(psi)/dn^2: for a "low" wall (neighbor at
        // +1, e.g. bottom/left) that's omega = 2(psi_wall-psi_nb)/h^2 +
        // 2*ut/h, and for a "high" wall (neighbor at -1, e.g. top/right)
        // it's the same first term but with a MINUS on the ut term. This
        // was flipped (-1 for "low", +1 for "high") -- invisible on the 3
        // stationary walls (ut=0 there always), but it reversed the sign
        // of the only nonzero-ut wall in this app, the lid: fluid right
        // under the lid came out moving opposite the lid itself, and the
        // whole cavity vortex spun the wrong way. Verified against the
        // corrected sign empirically (see the commit this fixes).
        double sign = (d.di == 1 || d.dj == 1) ? 1.0 : -1.0;
        sum += 2.0 * (psi_wall - psi_nb) / (d.h * d.h) + sign * 2.0 * d.ut / d.h;
    }
    return sum / static_cast<double>(dirs.size());
}

void NavierStokes2D::update_boundary_and_obstacle_vorticity() {
    double dx = dx_, dy = dy_;
    double U = cfg_.U;

    for (int j = 0; j < ny_; ++j)
        if (!solid_[idx2(j, 0, nx_)])
            omega_[idx2(j, 0, nx_)] = wall_omega(j, 0, {{0, 1, 0.0, dx}});
    for (int j = 0; j < ny_; ++j)
        if (!solid_[idx2(j, nx_ - 1, nx_)])
            omega_[idx2(j, nx_ - 1, nx_)] = wall_omega(j, nx_ - 1, {{0, -1, 0.0, dx}});
    for (int i = 0; i < nx_; ++i)
        if (!solid_[idx2(0, i, nx_)])
            omega_[idx2(0, i, nx_)] = wall_omega(0, i, {{1, 0, 0.0, dy}});
    double lid_u = (cfg_.kind == ScenarioKind2D::Cavity) ? U : 0.0;
    for (int i = 0; i < nx_; ++i)
        if (!solid_[idx2(ny_ - 1, i, nx_)])
            omega_[idx2(ny_ - 1, i, nx_)] = wall_omega(ny_ - 1, i, {{-1, 0, lid_u, dy}});

    bool any_solid = std::any_of(solid_.begin(), solid_.end(), [](std::uint8_t s) { return s != 0; });
    if (any_solid) {
        for (int j = 0; j < ny_; ++j) {
            for (int i = 0; i < nx_; ++i) {
                if (!solid_[idx2(j, i, nx_)]) continue;
                std::vector<WallDir> dirs;
                if (j + 1 < ny_ && !solid_[idx2(j + 1, i, nx_)]) dirs.push_back({1, 0, 0.0, dy});
                if (j - 1 >= 0 && !solid_[idx2(j - 1, i, nx_)]) dirs.push_back({-1, 0, 0.0, dy});
                if (i + 1 < nx_ && !solid_[idx2(j, i + 1, nx_)]) dirs.push_back({0, 1, 0.0, dx});
                if (i - 1 >= 0 && !solid_[idx2(j, i - 1, nx_)]) dirs.push_back({0, -1, 0.0, dx});
                if (!dirs.empty()) omega_[idx2(j, i, nx_)] = wall_omega(j, i, dirs);
            }
        }
    }

    if (is_channel_) {
        for (int j = 0; j < ny_; ++j) omega_[idx2(j, 0, nx_)] = 0.0; // inflow: irrotational uniform stream
        for (int j = 0; j < ny_; ++j)
            omega_[idx2(j, nx_ - 1, nx_)] = omega_[idx2(j, nx_ - 2, nx_)]; // outflow: zero-gradient
        for (int j = 0; j < ny_; ++j)
            psi_[idx2(j, nx_ - 1, nx_)] = 2 * psi_[idx2(j, nx_ - 2, nx_)] - psi_[idx2(j, nx_ - 3, nx_)];
        psi_[idx2(0, nx_ - 1, nx_)] = 0.0;
        psi_[idx2(ny_ - 1, nx_ - 1, nx_)] = Q_;
    }
}

void NavierStokes2D::compute_velocity() {
    std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    std::vector<double> u(n, 0.0), v(n, 0.0);

    for (int j = 1; j < ny_ - 1; ++j)
        for (int i = 0; i < nx_; ++i)
            u[idx2(j, i, nx_)] = (psi_[idx2(j + 1, i, nx_)] - psi_[idx2(j - 1, i, nx_)]) / (2 * dy_);
    for (int j = 0; j < ny_; ++j)
        for (int i = 1; i < nx_ - 1; ++i)
            v[idx2(j, i, nx_)] = -(psi_[idx2(j, i + 1, nx_)] - psi_[idx2(j, i - 1, nx_)]) / (2 * dx_);

    for (int i = 0; i < nx_; ++i) u[idx2(0, i, nx_)] = 0.0;
    double lid_or_zero = (cfg_.kind == ScenarioKind2D::Cavity) ? cfg_.U : 0.0;
    for (int i = 0; i < nx_; ++i) u[idx2(ny_ - 1, i, nx_)] = lid_or_zero;

    if (is_channel_) {
        for (int j = 0; j < ny_; ++j) {
            u[idx2(j, 0, nx_)] = cfg_.U;
            u[idx2(j, nx_ - 1, nx_)] = u[idx2(j, nx_ - 2, nx_)];
            v[idx2(j, 0, nx_)] = 0.0;
            v[idx2(j, nx_ - 1, nx_)] = v[idx2(j, nx_ - 2, nx_)];
        }
    }

    for (std::size_t k = 0; k < n; ++k) {
        if (solid_[k]) { u[k] = 0.0; v[k] = 0.0; }
    }

    u_ = std::move(u);
    v_ = std::move(v);
}

std::vector<double> NavierStokes2D::vorticity_transport() const {
    double dx = dx_, dy = dy_, Re = cfg_.Re;
    std::size_t n = static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_);
    std::vector<double> domega_dx(n, 0.0), domega_dy(n, 0.0), lap(n, 0.0), rhs(n, 0.0);

    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            double ui = u_[idx2(j, i, nx_)];
            double d = 0.0;
            if (i >= 1 && ui > 0.0) d += (omega_[idx2(j, i, nx_)] - omega_[idx2(j, i - 1, nx_)]) / dx;
            if (i <= nx_ - 2 && !(ui > 0.0)) d += (omega_[idx2(j, i + 1, nx_)] - omega_[idx2(j, i, nx_)]) / dx;
            domega_dx[idx2(j, i, nx_)] = d;
        }
    }
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            double vi = v_[idx2(j, i, nx_)];
            double d = 0.0;
            if (j >= 1 && vi > 0.0) d += (omega_[idx2(j, i, nx_)] - omega_[idx2(j - 1, i, nx_)]) / dy;
            if (j <= ny_ - 2 && !(vi > 0.0)) d += (omega_[idx2(j + 1, i, nx_)] - omega_[idx2(j, i, nx_)]) / dy;
            domega_dy[idx2(j, i, nx_)] = d;
        }
    }

    for (int j = 1; j < ny_ - 1; ++j) {
        for (int i = 1; i < nx_ - 1; ++i) {
            double c = omega_[idx2(j, i, nx_)];
            lap[idx2(j, i, nx_)] =
                (omega_[idx2(j, i + 1, nx_)] - 2 * c + omega_[idx2(j, i - 1, nx_)]) / (dx * dx)
              + (omega_[idx2(j + 1, i, nx_)] - 2 * c + omega_[idx2(j - 1, i, nx_)]) / (dy * dy);
        }
    }

    for (std::size_t k = 0; k < n; ++k) {
        rhs[k] = -(u_[k] * domega_dx[k] + v_[k] * domega_dy[k]) + lap[k] / Re;
    }
    return rhs;
}

double NavierStokes2D::solve_psi() {
    if (n_unknown_ == 0) return 0.0;

    double dx2 = dx_ * dx_, dy2 = dy_ * dy_;
    double cx = 1.0 / dx2, cy = 1.0 / dy2;

    Eigen::VectorXd b(n_unknown_);
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            std::size_t k_flat = idx2(j, i, nx_);
            if (!unknown_[k_flat]) continue;
            int k = node_index_[k_flat];
            b(k) = -omega_[k_flat];

            static constexpr int offsets[4][2] = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
            const double coeffs[4] = {cx, cx, cy, cy};
            for (int o = 0; o < 4; ++o) {
                int nj = j + offsets[o][0], ni = i + offsets[o][1];
                std::size_t nk_flat = idx2(nj, ni, nx_);
                if (fixed_[nk_flat]) b(k) -= coeffs[o] * psi_[nk_flat];
            }
        }
    }

    Eigen::VectorXd sol = lu_.solve(b);

    std::vector<double> new_psi = psi_;
    double residual = 0.0;
    for (int j = 0; j < ny_; ++j) {
        for (int i = 0; i < nx_; ++i) {
            std::size_t k_flat = idx2(j, i, nx_);
            if (!unknown_[k_flat]) continue;
            int k = node_index_[k_flat];
            double v = sol(k);
            residual = std::max(residual, std::fabs(v - psi_[k_flat]));
            new_psi[k_flat] = v;
        }
    }
    psi_ = std::move(new_psi);
    return residual;
}

double NavierStokes2D::step() {
    update_boundary_and_obstacle_vorticity();
    compute_velocity();
    std::vector<double> domega_dt = vorticity_transport();

    for (std::size_t k = 0; k < omega_.size(); ++k) omega_[k] += dt_ * domega_dt[k];
    for (std::size_t k = 0; k < omega_.size(); ++k)
        if (solid_[k]) omega_[k] = 0.0;

    double residual = solve_psi();
    compute_velocity();

    time_ += dt_;
    ++step_count_;
    last_residual_ = residual;
    return residual;
}

Fields2D NavierStokes2D::fields() const {
    Fields2D out;
    out.velocity_u = u_;
    out.velocity_v = v_;
    out.velocity_magnitude.resize(u_.size());
    for (std::size_t k = 0; k < u_.size(); ++k)
        out.velocity_magnitude[k] = std::sqrt(u_[k] * u_[k] + v_[k] * v_[k]);
    out.vorticity = omega_;
    out.streamfunction = psi_;
    out.obstacle.resize(solid_.size());
    for (std::size_t k = 0; k < solid_.size(); ++k) out.obstacle[k] = solid_[k] ? 1.0f : 0.0f;
    return out;
}

} // namespace cfd::solvers
