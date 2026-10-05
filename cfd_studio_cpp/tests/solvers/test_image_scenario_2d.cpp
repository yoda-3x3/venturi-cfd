#include <catch_amalgamated.hpp>

#include "solvers/image_scenario_2d.hpp"

using namespace cfd::solvers;

namespace {
// All hand-derived and independently cross-checked (see image_scenario_2d.hpp's
// own comment) via u=d(psi)/dy, v=-d(psi)/dx applied in each transformed
// frame -- not just asserted against whatever the implementation happens
// to produce.

std::vector<std::uint8_t> mask(std::initializer_list<int> vals) {
    std::vector<std::uint8_t> m;
    m.reserve(vals.size());
    for (int v : vals) m.push_back(static_cast<std::uint8_t>(v));
    return m;
}
} // namespace

// display_mask is 3 wide x 2 tall (W=3,H=2), j=0 (row-major first row) is
// the BOTTOM of the image:
//   j=1 (top):    0 1 0
//   j=0 (bottom): 1 0 0
TEST_CASE("orient_mask_for_solver: Right is a pure passthrough", "[solvers][image_scenario_2d]") {
    auto display = mask({1, 0, 0, 0, 1, 0});
    auto oriented = orient_mask_for_solver(display, 3, 2, ImageFlowDirection::Right);
    CHECK(oriented.nx == 3);
    CHECK(oriented.ny == 2);
    CHECK(oriented.mask == display);
}

TEST_CASE("orient_mask_for_solver: Left mirrors columns", "[solvers][image_scenario_2d]") {
    auto display = mask({1, 0, 0, 0, 1, 0});
    auto oriented = orient_mask_for_solver(display, 3, 2, ImageFlowDirection::Left);
    CHECK(oriented.nx == 3);
    CHECK(oriented.ny == 2);
    CHECK(oriented.mask == mask({0, 0, 1, 0, 1, 0}));
}

TEST_CASE("orient_mask_for_solver: Up transposes and swaps dims", "[solvers][image_scenario_2d]") {
    auto display = mask({1, 0, 0, 0, 1, 0});
    auto oriented = orient_mask_for_solver(display, 3, 2, ImageFlowDirection::Up);
    CHECK(oriented.nx == 2); // = display_ny
    CHECK(oriented.ny == 3); // = display_nx
    CHECK(oriented.mask == mask({1, 0, 0, 1, 0, 0}));
}

TEST_CASE("orient_mask_for_solver: Down transposes+flips and swaps dims", "[solvers][image_scenario_2d]") {
    auto display = mask({1, 0, 0, 0, 1, 0});
    auto oriented = orient_mask_for_solver(display, 3, 2, ImageFlowDirection::Down);
    CHECK(oriented.nx == 2);
    CHECK(oriented.ny == 3);
    CHECK(oriented.mask == mask({0, 1, 1, 0, 0, 0}));
}

namespace {
// A solver-space Fields2D where every field encodes its own (j,i) so a
// transform bug shows up as values coming from the wrong cell, not just
// the wrong sign. nx/ny match whichever test constructs this.
Fields2D makeEncodedFields(int nx, int ny) {
    Fields2D f;
    std::size_t n = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
    f.velocity_u.resize(n);
    f.velocity_v.resize(n);
    f.velocity_magnitude.resize(n);
    f.vorticity.resize(n);
    f.streamfunction.resize(n);
    f.obstacle.resize(n);
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            double code = 10.0 * j + i;
            std::size_t k = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i);
            f.velocity_u[k] = code;
            f.velocity_v[k] = 100.0 + code;
            f.vorticity[k] = 1000.0 + code;
            f.streamfunction[k] = 5000.0 + code;
            f.velocity_magnitude[k] = 9000.0 + code;
            f.obstacle[k] = static_cast<float>(code);
        }
    }
    return f;
}
} // namespace

TEST_CASE("orient_fields_for_display: Right is a pure passthrough", "[solvers][image_scenario_2d]") {
    auto sf = makeEncodedFields(3, 2);
    int dnx = 0, dny = 0;
    auto out = orient_fields_for_display(sf, 3, 2, ImageFlowDirection::Right, dnx, dny);
    CHECK(dnx == 3);
    CHECK(dny == 2);
    CHECK(out.velocity_u == sf.velocity_u);
    CHECK(out.velocity_v == sf.velocity_v);
    CHECK(out.vorticity == sf.vorticity);
    CHECK(out.streamfunction == sf.streamfunction);
}

// Left: an improper (mirror) transform -- u, vorticity, and streamfunction
// negate; v does not.
TEST_CASE("orient_fields_for_display: Left mirrors and negates u/vorticity/psi", "[solvers][image_scenario_2d]") {
    auto sf = makeEncodedFields(3, 2);
    int dnx = 0, dny = 0;
    auto out = orient_fields_for_display(sf, 3, 2, ImageFlowDirection::Left, dnx, dny);
    CHECK(dnx == 3);
    CHECK(dny == 2);

    auto at = [&](const std::vector<double>& v, int j, int i) { return v[static_cast<std::size_t>(j) * 3 + static_cast<std::size_t>(i)]; };

    CHECK(at(out.velocity_u, 0, 0) == Catch::Approx(-2.0));
    CHECK(at(out.velocity_v, 0, 0) == Catch::Approx(102.0));
    CHECK(at(out.vorticity, 0, 0) == Catch::Approx(-1002.0));
    CHECK(at(out.streamfunction, 0, 0) == Catch::Approx(-5002.0));
    CHECK(at(out.velocity_magnitude, 0, 0) == Catch::Approx(9002.0));

    CHECK(at(out.velocity_u, 1, 1) == Catch::Approx(-11.0));
    CHECK(at(out.velocity_v, 1, 1) == Catch::Approx(111.0));
    CHECK(at(out.vorticity, 1, 1) == Catch::Approx(-1011.0));
    CHECK(at(out.streamfunction, 1, 1) == Catch::Approx(-5011.0));
}

// Up: also improper (a coordinate transpose) -- u/v swap (not negated) and
// vorticity/streamfunction negate.
TEST_CASE("orient_fields_for_display: Up swaps u/v and negates vorticity/psi", "[solvers][image_scenario_2d]") {
    auto sf = makeEncodedFields(/*nx=*/2, /*ny=*/3);
    int dnx = 0, dny = 0;
    auto out = orient_fields_for_display(sf, 2, 3, ImageFlowDirection::Up, dnx, dny);
    CHECK(dnx == 3);
    CHECK(dny == 2);

    auto at = [&](const std::vector<double>& v, int j, int i) { return v[static_cast<std::size_t>(j) * 3 + static_cast<std::size_t>(i)]; };

    // display[0][1] <- solver(js=1,is=0) = code 10
    CHECK(at(out.velocity_u, 0, 1) == Catch::Approx(110.0));  // = v_solver
    CHECK(at(out.velocity_v, 0, 1) == Catch::Approx(10.0));   // = u_solver
    CHECK(at(out.vorticity, 0, 1) == Catch::Approx(-1010.0));
    CHECK(at(out.streamfunction, 0, 1) == Catch::Approx(-5010.0));

    // display[1][2] <- solver(js=2,is=1) = code 21
    CHECK(at(out.velocity_u, 1, 2) == Catch::Approx(121.0));
    CHECK(at(out.velocity_v, 1, 2) == Catch::Approx(21.0));
    CHECK(at(out.vorticity, 1, 2) == Catch::Approx(-1021.0));
    CHECK(at(out.streamfunction, 1, 2) == Catch::Approx(-5021.0));
}

// Down: a PROPER 90-degree rotation (transpose+flip composed) -- u/v swap
// with v negated, vorticity/streamfunction unchanged (pseudo-scalars are
// invariant under a proper rotation, unlike Left/Up's mirrors).
TEST_CASE("orient_fields_for_display: Down swaps u/v (v negated), vorticity/psi unchanged", "[solvers][image_scenario_2d]") {
    auto sf = makeEncodedFields(/*nx=*/2, /*ny=*/3);
    int dnx = 0, dny = 0;
    auto out = orient_fields_for_display(sf, 2, 3, ImageFlowDirection::Down, dnx, dny);
    CHECK(dnx == 3);
    CHECK(dny == 2);

    auto at = [&](const std::vector<double>& v, int j, int i) { return v[static_cast<std::size_t>(j) * 3 + static_cast<std::size_t>(i)]; };

    // display[0][0] <- solver(js=0,is=1) = code 1
    CHECK(at(out.velocity_u, 0, 0) == Catch::Approx(101.0));  // = v_solver
    CHECK(at(out.velocity_v, 0, 0) == Catch::Approx(-1.0));   // = -u_solver
    CHECK(at(out.vorticity, 0, 0) == Catch::Approx(1001.0));  // unchanged
    CHECK(at(out.streamfunction, 0, 0) == Catch::Approx(5001.0));

    // display[1][1] <- solver(js=1,is=0) = code 10
    CHECK(at(out.velocity_u, 1, 1) == Catch::Approx(110.0));
    CHECK(at(out.velocity_v, 1, 1) == Catch::Approx(-10.0));
    CHECK(at(out.vorticity, 1, 1) == Catch::Approx(1010.0));
    CHECK(at(out.streamfunction, 1, 1) == Catch::Approx(5010.0));
}
