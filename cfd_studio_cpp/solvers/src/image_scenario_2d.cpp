#include "solvers/image_scenario_2d.hpp"

namespace cfd::solvers {

namespace {
inline std::size_t flat(int j, int i, int nx) {
    return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i);
}
} // namespace

OrientedMask2D orient_mask_for_solver(const std::vector<std::uint8_t>& display_mask, int display_nx,
                                       int display_ny, ImageFlowDirection dir) {
    const int W = display_nx, H = display_ny;
    OrientedMask2D out;

    switch (dir) {
        case ImageFlowDirection::Right: {
            out.nx = W;
            out.ny = H;
            out.mask = display_mask;
            break;
        }
        case ImageFlowDirection::Left: {
            out.nx = W;
            out.ny = H;
            out.mask.resize(static_cast<std::size_t>(W) * static_cast<std::size_t>(H));
            for (int j = 0; j < H; ++j)
                for (int i = 0; i < W; ++i)
                    out.mask[flat(j, i, W)] = display_mask[flat(j, W - 1 - i, W)];
            break;
        }
        case ImageFlowDirection::Up: {
            // Solver's flow axis (i_s) becomes the display's vertical
            // extent; its own "vertical" (j_s) becomes the display's
            // horizontal extent.
            out.nx = H;
            out.ny = W;
            out.mask.resize(static_cast<std::size_t>(H) * static_cast<std::size_t>(W));
            for (int js = 0; js < W; ++js)
                for (int is = 0; is < H; ++is)
                    out.mask[flat(js, is, H)] = display_mask[flat(is, js, W)];
            break;
        }
        case ImageFlowDirection::Down: {
            out.nx = H;
            out.ny = W;
            out.mask.resize(static_cast<std::size_t>(H) * static_cast<std::size_t>(W));
            for (int js = 0; js < W; ++js)
                for (int is = 0; is < H; ++is)
                    out.mask[flat(js, is, H)] = display_mask[flat(H - 1 - is, js, W)];
            break;
        }
    }
    return out;
}

Fields2D orient_fields_for_display(const Fields2D& sf, int solverNx, int solverNy, ImageFlowDirection dir,
                                    int& display_nx, int& display_ny) {
    Fields2D out;

    auto allocate = [&](int nx, int ny) {
        std::size_t n = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
        out.velocity_u.resize(n);
        out.velocity_v.resize(n);
        out.velocity_magnitude.resize(n);
        out.vorticity.resize(n);
        out.streamfunction.resize(n);
        out.obstacle.resize(n);
    };

    switch (dir) {
        case ImageFlowDirection::Right: {
            display_nx = solverNx;
            display_ny = solverNy;
            return sf; // native orientation -- pure passthrough
        }
        case ImageFlowDirection::Left: {
            int W = solverNx, H = solverNy;
            display_nx = W;
            display_ny = H;
            allocate(W, H);
            for (int j = 0; j < H; ++j) {
                for (int i = 0; i < W; ++i) {
                    std::size_t d = flat(j, i, W);
                    std::size_t s = flat(j, W - 1 - i, W);
                    out.velocity_u[d] = -sf.velocity_u[s];
                    out.velocity_v[d] = sf.velocity_v[s];
                    out.velocity_magnitude[d] = sf.velocity_magnitude[s];
                    out.vorticity[d] = -sf.vorticity[s];
                    out.streamfunction[d] = -sf.streamfunction[s];
                    out.obstacle[d] = sf.obstacle[s];
                }
            }
            break;
        }
        case ImageFlowDirection::Up: {
            // solver: nx=H(display_ny), ny=W(display_nx) -- see orient_mask_for_solver.
            int H = solverNx, W = solverNy;
            display_nx = W;
            display_ny = H;
            allocate(W, H);
            for (int j = 0; j < H; ++j) {
                for (int i = 0; i < W; ++i) {
                    std::size_t d = flat(j, i, W);
                    std::size_t s = flat(/*js=*/i, /*is=*/j, H);
                    out.velocity_u[d] = sf.velocity_v[s];
                    out.velocity_v[d] = sf.velocity_u[s];
                    out.velocity_magnitude[d] = sf.velocity_magnitude[s];
                    out.vorticity[d] = -sf.vorticity[s];
                    out.streamfunction[d] = -sf.streamfunction[s];
                    out.obstacle[d] = sf.obstacle[s];
                }
            }
            break;
        }
        case ImageFlowDirection::Down: {
            int H = solverNx, W = solverNy;
            display_nx = W;
            display_ny = H;
            allocate(W, H);
            for (int j = 0; j < H; ++j) {
                for (int i = 0; i < W; ++i) {
                    std::size_t d = flat(j, i, W);
                    std::size_t s = flat(/*js=*/i, /*is=*/H - 1 - j, H);
                    out.velocity_u[d] = sf.velocity_v[s];
                    out.velocity_v[d] = -sf.velocity_u[s];
                    out.velocity_magnitude[d] = sf.velocity_magnitude[s];
                    out.vorticity[d] = sf.vorticity[s];
                    out.streamfunction[d] = sf.streamfunction[s];
                    out.obstacle[d] = sf.obstacle[s];
                }
            }
            break;
        }
    }
    return out;
}

} // namespace cfd::solvers
