#include "pipeline/run_2d.hpp"

#include <chrono>
#include <stdexcept>

#include "io/vtk_writer.hpp"
#include "solvers/navier_stokes_2d.hpp"
#include "solvers/scenario_presets_2d.hpp"

namespace cfd::pipeline {

namespace {
constexpr double kPreviewThrottleSeconds = 0.15;
}

Run2DResult run_2d(const Run2DOptions& opts, const std::function<void(int, double)>& on_progress,
                    const Preview2DCallback& on_preview) {
    if (opts.output_every <= 0) throw std::invalid_argument("run_2d: output_every must be positive");

    const bool is_custom_image = opts.scenario == "custom_image";

    cfd::solvers::SolverConfig2D config;
    if (is_custom_image) {
        if (!opts.custom_mask) throw std::invalid_argument("run_2d: custom_image scenario requires custom_mask");
        config.nx = opts.custom_mask_nx;
        config.ny = opts.custom_mask_ny;
        config.Lx = 1.0;
        config.Ly = static_cast<double>(config.ny) / static_cast<double>(config.nx); // keep grid cells square
        config.Re = opts.Re.value_or(100.0);
        config.kind = cfd::solvers::ScenarioKind2D::ImageObstacle;
        config.U = opts.U.value_or(1.0);
        config.custom_solid_mask = *opts.custom_mask;
        config.dt = opts.dt;
    } else {
        const auto& preset = cfd::solvers::scenario_preset_2d(opts.scenario);
        config.nx = opts.nx.value_or(preset.default_nx);
        config.ny = opts.ny.value_or(preset.default_ny);
        config.Lx = preset.Lx;
        config.Ly = preset.Ly;
        config.Re = opts.Re.value_or(preset.default_Re);
        config.kind = preset.kind;
        config.U = opts.U.value_or(preset.default_U);
        config.dt = opts.dt;
        if (preset.has_obstacle) {
            auto def = cfd::solvers::default_obstacle_2d(preset);
            cfd::solvers::Obstacle2D obstacle;
            obstacle.x0 = opts.obstacle_x0.value_or(def.x0);
            obstacle.width = opts.obstacle_width.value_or(def.width);
            obstacle.height = opts.obstacle_height.value_or(def.height);
            config.obstacle = obstacle;
        }
    }

    cfd::solvers::NavierStokes2D solver(config);

    // For custom_image, everything downstream (preview + the written VTK
    // series) should show the flow in the user's own uploaded-image
    // orientation, not the solver's own always-left-to-right internal
    // frame -- see orient_fields_for_display(). displayNx/displayNy differ
    // from config.nx/config.ny exactly when image_direction is Up/Down
    // (the solver's axes are transposed relative to the display's).
    int displayNx = config.nx, displayNy = config.ny;
    auto oriented_fields = [&]() -> cfd::solvers::Fields2D {
        auto fields = solver.fields();
        if (!is_custom_image) return fields;
        return cfd::solvers::orient_fields_for_display(fields, config.nx, config.ny, opts.image_direction,
                                                         displayNx, displayNy);
    };

    std::string case_name = opts.case_name.empty() ? opts.scenario : opts.case_name;
    cfd::io::VtkSeriesWriter writer(opts.output_dir, case_name);
    writer.clear();

    auto write_step = [&](int step_index) {
        auto fields = oriented_fields();
        writer.write_step(step_index, solver.time(), displayNx, displayNy, solver.dx(), solver.dy(),
                           fields.velocity_u, fields.velocity_v, fields.velocity_magnitude,
                           fields.vorticity, fields.streamfunction, fields.obstacle);
    };

    write_step(0);

    Run2DResult result;
    result.pvd_path = writer.pvd_path();

    auto last_update = std::chrono::steady_clock::now();
    for (int step = 1; step <= opts.n_steps; ++step) {
        if (opts.stop_flag && opts.stop_flag->load()) {
            result.stopped = true;
            break;
        }

        result.final_residual = solver.step();
        result.steps_run = step;

        auto now = std::chrono::steady_clock::now();
        bool throttled_tick =
            std::chrono::duration<double>(now - last_update).count() >= kPreviewThrottleSeconds || step == opts.n_steps;
        if (throttled_tick) {
            last_update = now;
            if (on_progress) on_progress(step, result.final_residual);
            if (on_preview) on_preview(oriented_fields(), displayNx, displayNy, solver.dx(), solver.dy());
        }

        if (step % opts.output_every == 0 || step == opts.n_steps) {
            write_step(step);
        }
    }

    return result;
}

} // namespace cfd::pipeline
