#include "results_viewer_widget.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QColor>
#include <QMouseEvent>
#include <QVector2D>
#include <QWheelEvent>

#include "colormap.hpp"

namespace {
constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;

// Unlike MeshPreviewWidget's identical-looking shader (a separate copy in
// mesh_preview_widget.cpp, which has no field data to show), this version
// takes a per-vertex color -- the object surface is shaded by the current
// scalar field sampled just off the wall (see rebuildMeshGeometry), not a
// single flat theme color, to match how CFD surface renders typically
// look (a colored, lit body, not a solid-tint one).
const char* kMeshVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uMvp;
out vec3 vNormal;
out vec3 vColor;
void main() {
    vNormal = aNormal;
    vColor = aColor;
    gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

const char* kMeshFragmentShader = R"(
#version 330 core
in vec3 vNormal;
in vec3 vColor;
uniform vec3 uLightDir;
out vec4 fragColor;
void main() {
    vec3 n = normalize(vNormal);
    float diff = max(dot(n, normalize(-uLightDir)), 0.0);
    vec3 color = vColor * (0.35 + 0.65 * diff);
    fragColor = vec4(color, 1.0);
}
)";

const char* kSliceVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUv;
uniform mat4 uMvp;
out vec2 vUv;
void main() {
    vUv = aUv;
    gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

// Reproduces PlotWidget::colormapSample()'s exact 5-stop viridis-like LUT
// (app/src/widgets/plot_widget.cpp) via a cascading-mix chain, so the
// slice plane reads consistently with the app's existing 2D heatmaps.
// Cascading clamp(t4-i,0,1) mixes is a standard piecewise-linear
// multi-stop gradient technique: at any t4 in [i,i+1], every mix before
// the i-th has already fully applied (clamp==1) and every one after is a
// no-op (clamp==0), leaving exactly mix(c_i, c_{i+1}, frac) as the result
// -- chosen over uniform-array indexing (uStops[i]) to sidestep any
// question of dynamic uniform-array-index support on older GL 3.3
// drivers.
const char* kSliceFragmentShader = R"(
#version 330 core
in vec2 vUv;
uniform sampler2D uField;
uniform vec2 uValueRange;
out vec4 fragColor;
vec3 colormap(float t) {
    const vec3 c0 = vec3(0.26667, 0.00392, 0.32941);
    const vec3 c1 = vec3(0.23137, 0.32157, 0.54510);
    const vec3 c2 = vec3(0.12941, 0.56863, 0.54902);
    const vec3 c3 = vec3(0.36863, 0.78824, 0.38431);
    const vec3 c4 = vec3(0.99216, 0.90588, 0.14510);
    float t4 = clamp(t, 0.0, 1.0) * 4.0;
    vec3 col = c0;
    col = mix(col, c1, clamp(t4 - 0.0, 0.0, 1.0));
    col = mix(col, c2, clamp(t4 - 1.0, 0.0, 1.0));
    col = mix(col, c3, clamp(t4 - 2.0, 0.0, 1.0));
    col = mix(col, c4, clamp(t4 - 3.0, 0.0, 1.0));
    return col;
}
void main() {
    float raw = texture(uField, vUv).r;
    float range = max(uValueRange.y - uValueRange.x, 1e-12);
    float t = (raw - uValueRange.x) / range;
    fragColor = vec4(colormap(t), 1.0);
}
)";

// Shared by streamlines (GL_LINE_STRIP) and arrow glyphs (GL_LINES) --
// both are just per-vertex-colored line segments.
const char* kVectorVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 uMvp;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

const char* kVectorFragmentShader = R"(
#version 330 core
in vec3 vColor;
out vec4 fragColor;
void main() {
    fragColor = vec4(vColor, 1.0);
}
)";

// Proxy geometry for the vortex-volume ray marcher: a cube spanning the
// domain bounds. Just passes its own (domain-space) position through --
// the fragment shader uses it only to know which screen pixels the box
// could possibly cover; the actual march bounds come from an analytic
// ray-box test against uBoxMin/uBoxMax, not from which face got
// rasterized, so this works whether the camera is inside or outside the
// box without needing any special face-culling trick.
const char* kVolumeVertexShader = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uMvp;
out vec3 vWorldPos;
void main() {
    vWorldPos = aPos;
    gl_Position = uMvp * vec4(aPos, 1.0);
}
)";

// Front-to-back alpha-composited ray march through the Q-criterion 3D
// texture. Colored with a multi-band "shaded isosurface" transfer function
// (pale green -> blue -> violet -> orange -> red as density rises) plus a
// gradient-estimated normal for lightweight diffuse/specular shading, so
// the volume reads as distinct, lit vortex lobes -- the classic layered
// look of a multi-isosurface CFD render -- rather than a flat, single-hue
// haze. Distinct from flow_colormap_sample()'s rainbow (used by
// streamlines/arrows), which is a 2D line/arrow palette, not a volumetric
// shading model.
const char* kVolumeFragmentShader = R"(
#version 330 core
in vec3 vWorldPos;
uniform vec3 uCameraPos;
uniform vec3 uBoxMin;
uniform vec3 uBoxMax;
uniform sampler3D uVolume;
out vec4 fragColor;

// The CPU side uploads the (i-slowest/k-fastest) Q field with GL
// width=nz, height=ny, depth=nx -- see rebuildVortexVolume() -- so texture
// axes are (s,t,r) = (zFrac, yFrac, xFrac); every sample goes through this
// helper so the gradient taps below stay consistent with it.
float sampleDensity(vec3 frac) {
    return texture(uVolume, vec3(frac.z, frac.y, frac.x)).r;
}

// Ordered by rising Q-criterion magnitude, not physical meaning (this is
// still a single-field render) -- chosen to band visually the way a
// multi-isosurface CFD figure does: a soft, broad low-level shell reading
// as pale green, tightening through blue and violet, up to a hot,
// near-opaque orange/red core at the strongest rotation.
vec3 bandColormap(float t) {
    const vec3 cGreen  = vec3(0.45, 0.95, 0.40);
    const vec3 cBlue   = vec3(0.15, 0.60, 1.00);
    const vec3 cViolet = vec3(0.68, 0.30, 0.92);
    const vec3 cOrange = vec3(1.00, 0.58, 0.10);
    const vec3 cRed    = vec3(0.98, 0.12, 0.10);
    float t4 = clamp(t, 0.0, 1.0) * 4.0;
    // smoothstep, not a hard clamp() ramp -- a linear mix() has a slope
    // discontinuity right where each band's clamp() hits 0 or 1, and a
    // ray sweeping through that seam at a shallow angle turns that kink
    // into a visible contour line (this is what the first version's
    // "banding artifact" screenshots actually were, not a texture/step
    // sampling issue).
    vec3 col = cGreen;
    col = mix(col, cBlue,   smoothstep(0.0, 1.0, clamp(t4 - 0.0, 0.0, 1.0)));
    col = mix(col, cViolet, smoothstep(0.0, 1.0, clamp(t4 - 1.0, 0.0, 1.0)));
    col = mix(col, cOrange, smoothstep(0.0, 1.0, clamp(t4 - 2.0, 0.0, 1.0)));
    col = mix(col, cRed,    smoothstep(0.0, 1.0, clamp(t4 - 3.0, 0.0, 1.0)));
    return col;
}

void main() {
    vec3 rayDir = normalize(vWorldPos - uCameraPos);
    vec3 invDir = 1.0 / rayDir;
    vec3 t0s = (uBoxMin - uCameraPos) * invDir;
    vec3 t1s = (uBoxMax - uCameraPos) * invDir;
    vec3 tsMin = min(t0s, t1s);
    vec3 tsMax = max(t0s, t1s);
    float tNear = max(max(tsMin.x, tsMin.y), tsMin.z);
    float tFar = min(min(tsMax.x, tsMax.y), tsMax.z);
    tNear = max(tNear, 0.0); // camera may be inside the box
    if (tNear >= tFar) discard;

    const int kSteps = 160;
    float stepSize = (tFar - tNear) / float(kSteps);
    vec3 boxSize = uBoxMax - uBoxMin;
    vec3 viewDir = -rayDir;
    // Fixed key light, not tied to the camera -- so the shaded lobes hold
    // still and read as physical shapes while the user orbits, instead of
    // relighting (and thus appearing to change shape) every frame.
    vec3 lightDir = normalize(vec3(0.45, 0.82, 0.35));

    vec4 accum = vec4(0.0);
    for (int i = 0; i < kSteps; ++i) {
        float t = tNear + (float(i) + 0.5) * stepSize;
        vec3 p = uCameraPos + rayDir * t;
        vec3 frac = (p - uBoxMin) / boxSize;
        float density = sampleDensity(frac);
        // Measured against a real run (see the commit this replaced): the
        // positive-Q population is extremely right-skewed -- only the top
        // ~2% carries real signal, and even the loosest useful threshold
        // only forms tight runs of cells, not a broad diffuse haze. Kept
        // deliberately close to that finding (a slightly lower floor than
        // before, to give the outer "green shell" band room to be visible
        // at low opacity) rather than reopening the earlier oversaturation
        // bug.
        float shaped = clamp((density - 0.30) / 0.70, 0.0, 1.0);
        if (shaped <= 0.0) continue;

        // Lightweight surface shading: estimate the local density
        // gradient via central differences and treat its (inverted, since
        // density falls outward) direction as a normal. Only costs extra
        // samples on the already-tight qualifying region above, not the
        // whole ray. e is sized to a few grid cells (not sub-cell) since a
        // tap smaller than the source Q field's own voxel spacing just
        // samples trilinear-interpolation noise within one cell instead of
        // the field's real shape, which reads as a noisy, undefined
        // surface rather than a smoothly shaded one.
        float e = 0.02;
        float gx = sampleDensity(frac + vec3(e, 0.0, 0.0)) - sampleDensity(frac - vec3(e, 0.0, 0.0));
        float gy = sampleDensity(frac + vec3(0.0, e, 0.0)) - sampleDensity(frac - vec3(0.0, e, 0.0));
        float gz = sampleDensity(frac + vec3(0.0, 0.0, e)) - sampleDensity(frac - vec3(0.0, 0.0, e));
        vec3 grad = vec3(gx, gy, gz) / boxSize;
        vec3 normal = length(grad) > 1e-8 ? normalize(-grad) : viewDir;

        float diffuse = max(dot(normal, lightDir), 0.0);
        vec3 halfVec = normalize(lightDir + viewDir);
        float spec = pow(max(dot(normal, halfVec), 0.0), 28.0);
        float rim = pow(1.0 - max(dot(normal, viewDir), 0.0), 3.0);

        vec3 baseColor = bandColormap(shaped);
        // Shading floor kept high (0.62..1.0, not 0.4..1.0) so the band
        // hue itself stays legible even where the gradient normal is
        // noisy or grazing -- the earlier, deeper floor was why the first
        // version's screenshots read as muddy/desaturated rather than the
        // vivid reference colors.
        vec3 shadedColor = baseColor * (0.62 + 0.38 * diffuse) + vec3(1.0) * spec * 0.35 + baseColor * rim * 0.3;

        // Steeper power curve than a plain Beer-Lambert law so the bands
        // read as near-solid shaded shells (matching the reference look)
        // rather than a smooth volumetric gradient -- opacity still scales
        // with physical distance crossed (via stepSize), just ramping up
        // faster with density. Gentler exponent and higher coefficient
        // than the first pass: shaped values above threshold now reach
        // full opacity sooner, so lobes read as solid rather than
        // translucent smoke over the dark background.
        float bandAlpha = pow(shaped, 1.2);
        float alpha = 1.0 - exp(-bandAlpha * 4.0 * stepSize);

        accum.rgb += (1.0 - accum.a) * shadedColor * alpha;
        accum.a += (1.0 - accum.a) * alpha;
        if (accum.a > 0.98) break;
    }
    if (accum.a < 0.01) discard;
    fragColor = accum;
}
)";
} // namespace

ResultsViewerWidget::ResultsViewerWidget(QWidget* parent) : QOpenGLWidget(parent) {
    setMouseTracking(true);
}

ResultsViewerWidget::~ResultsViewerWidget() {
    makeCurrent();
    meshVbo_.destroy();
    sliceVbo_.destroy();
    streamlineVbo_.destroy();
    arrowVbo_.destroy();
    volumeBoxVbo_.destroy();
    if (sliceTexture_ != 0) glDeleteTextures(1, &sliceTexture_);
    if (volumeTexture_ != 0) glDeleteTextures(1, &volumeTexture_);
    doneCurrent();
}

void ResultsViewerWidget::setTheme(const Theme& theme) {
    theme_ = theme;
    update();
}

void ResultsViewerWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    meshProgram_ = std::make_unique<QOpenGLShaderProgram>();
    meshProgram_->addShaderFromSourceCode(QOpenGLShader::Vertex, kMeshVertexShader);
    meshProgram_->addShaderFromSourceCode(QOpenGLShader::Fragment, kMeshFragmentShader);
    meshProgram_->link();

    sliceProgram_ = std::make_unique<QOpenGLShaderProgram>();
    sliceProgram_->addShaderFromSourceCode(QOpenGLShader::Vertex, kSliceVertexShader);
    sliceProgram_->addShaderFromSourceCode(QOpenGLShader::Fragment, kSliceFragmentShader);
    sliceProgram_->link();

    vectorProgram_ = std::make_unique<QOpenGLShaderProgram>();
    vectorProgram_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVectorVertexShader);
    vectorProgram_->addShaderFromSourceCode(QOpenGLShader::Fragment, kVectorFragmentShader);
    vectorProgram_->link();

    volumeProgram_ = std::make_unique<QOpenGLShaderProgram>();
    volumeProgram_->addShaderFromSourceCode(QOpenGLShader::Vertex, kVolumeVertexShader);
    volumeProgram_->addShaderFromSourceCode(QOpenGLShader::Fragment, kVolumeFragmentShader);
    volumeProgram_->link();

    meshVao_.create();
    sliceVao_.create();
    sliceVbo_.create();
    streamlineVao_.create();
    streamlineVbo_.create();
    arrowVao_.create();
    arrowVbo_.create();
    volumeBoxVao_.create();
    volumeBoxVbo_.create();

    glGenTextures(1, &sliceTexture_);
    glBindTexture(GL_TEXTURE_2D, sliceTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenTextures(1, &volumeTexture_);
    glBindTexture(GL_TEXTURE_3D, volumeTexture_);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

void ResultsViewerWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void ResultsViewerWidget::setMesh(const cfd::mesh::Mesh& mesh) {
    mesh_ = mesh;
    haveMesh_ = true;
    meshDirty_ = true;

    auto bounds = mesh_.bounds();
    cfd::mesh::Vec3 c = bounds.center();
    center_ = QVector3D(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z));
    cfd::mesh::Vec3 ext = bounds.extents();
    radius_ = static_cast<float>(std::max({ext.x, ext.y, ext.z, 1e-6})) * 0.6f;
    distance_ = radius_ * 3.0f;
    yaw_ = -45.0f;
    pitch_ = 25.0f;

    update();
}

void ResultsViewerWidget::setResultsCache(std::shared_ptr<cfd::io::ResultsCacheReader> reader) {
    reader_ = std::move(reader);
    haveFrame_ = false;
    setFrame(0);
}

int ResultsViewerWidget::frameCount() const {
    return reader_ ? reader_->frame_count() : 0;
}

double ResultsViewerWidget::frameTime(int index) const {
    return reader_ ? reader_->frame_time(index) : 0.0;
}

void ResultsViewerWidget::setFrame(int index) {
    if (!reader_ || index < 0 || index >= reader_->frame_count()) return;
    // A load failure here (e.g. a truncated frame file) is treated as a
    // no-op rather than propagated -- this can be called from a slider's
    // valueChanged signal, and letting an exception cross back out through
    // Qt's event dispatch is not something to risk over one bad frame.
    try {
        currentFrame_ = reader_->load_frame(index);
    } catch (const std::exception&) {
        return;
    }
    frameIndex_ = index;
    haveFrame_ = true;
    meshDirty_ = true; // surface color samples the current field/frame
    sliceDirty_ = true;
    streamlinesDirty_ = true; // velocity field changed
    arrowsDirty_ = true;
    volumeDirty_ = true; // Q-criterion depends on the velocity field
    update();
}

void ResultsViewerWidget::setScalarField(ScalarField field) {
    field_ = field;
    meshDirty_ = true; // surface color samples the current field
    sliceDirty_ = true;
    update();
}

void ResultsViewerWidget::setSliceAxis(Axis axis) {
    axis_ = axis;
    sliceDirty_ = true;
    arrowsDirty_ = true; // arrows are sampled on the slice plane's own grid
    update();
}

void ResultsViewerWidget::setSlicePosition(double t) {
    slicePosition_ = std::clamp(t, 0.0, 1.0);
    sliceDirty_ = true;
    arrowsDirty_ = true;
    update();
}

void ResultsViewerWidget::setShowStreamlines(bool show) {
    showStreamlines_ = show;
    if (show) streamlinesDirty_ = true;
    update();
}

void ResultsViewerWidget::setShowVelocityArrows(bool show) {
    showArrows_ = show;
    if (show) arrowsDirty_ = true;
    update();
}

void ResultsViewerWidget::setVectorDensity(int density) {
    vectorDensity_ = std::clamp(density, 2, 60);
    streamlinesDirty_ = true;
    arrowsDirty_ = true;
    update();
}

void ResultsViewerWidget::setShowVortexVolume(bool show) {
    showVortexVolume_ = show;
    if (show) volumeDirty_ = true;
    update();
}

void ResultsViewerWidget::rebuildMeshGeometry() {
    // Surface color comes from the active scalar field sampled a couple of
    // cells off the wall along the face normal, not right on it: a no-slip
    // wall has ~zero velocity exactly at the surface, so sampling in place
    // would paint the whole body a single flat color for any velocity
    // field. Offsetting into the adjacent fluid captures the actual
    // near-wall variation (e.g. the visible over-the-body speedup CFD
    // renders typically show).
    bool haveField = haveFrame_ && reader_;
    const std::vector<double>* field = haveField ? &fieldArray(field_) : nullptr;
    float offset = haveField
        ? static_cast<float>(2.0 * std::min({reader_->dx(), reader_->dy(), reader_->dz()}))
        : 0.0f;

    struct VertexScratch {
        cfd::mesh::Vec3 pos;
        cfd::mesh::Vec3 normal;
        float value;
    };
    std::vector<VertexScratch> verts;
    verts.reserve(mesh_.triangles.size() * 3);

    float vmin = std::numeric_limits<float>::infinity(), vmax = -std::numeric_limits<float>::infinity();
    for (const auto& tri : mesh_.triangles) {
        const auto& a = mesh_.vertices[tri[0]];
        const auto& b = mesh_.vertices[tri[1]];
        const auto& c = mesh_.vertices[tri[2]];
        cfd::mesh::Vec3 normal = cfd::mesh::normalize(cfd::mesh::cross(b - a, c - a));
        for (const cfd::mesh::Vec3* v : {&a, &b, &c}) {
            float value = 0.0f;
            if (field) {
                value = static_cast<float>(sampleScalar(*field, v->x + normal.x * offset, v->y + normal.y * offset,
                                                          v->z + normal.z * offset));
                vmin = std::min(vmin, value);
                vmax = std::max(vmax, value);
            }
            verts.push_back({*v, normal, value});
        }
    }
    float range = (vmax > vmin) ? (vmax - vmin) : 1.0f;
    QColor fallback(theme_.accent); // shown before any results frame is loaded

    std::vector<float> vertexData;
    vertexData.reserve(verts.size() * 9);
    for (const auto& vs : verts) {
        QColor c = field ? flow_colormap_sample((vs.value - vmin) / range) : fallback;
        vertexData.insert(vertexData.end(), {static_cast<float>(vs.pos.x), static_cast<float>(vs.pos.y),
                                              static_cast<float>(vs.pos.z), static_cast<float>(vs.normal.x),
                                              static_cast<float>(vs.normal.y), static_cast<float>(vs.normal.z),
                                              static_cast<float>(c.redF()), static_cast<float>(c.greenF()),
                                              static_cast<float>(c.blueF())});
    }
    meshVertexCount_ = static_cast<int>(verts.size());

    meshVao_.bind();
    if (!meshVbo_.isCreated()) meshVbo_.create();
    meshVbo_.bind();
    meshVbo_.allocate(vertexData.data(), static_cast<int>(vertexData.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    meshVbo_.release();
    meshVao_.release();

    meshDirty_ = false;
}

void ResultsViewerWidget::rebuildSlice() {
    sliceDirty_ = false;
    if (!haveFrame_ || !reader_) return;

    int nx = reader_->nx(), ny = reader_->ny(), nz = reader_->nz();
    double Lx = nx * reader_->dx(), Ly = ny * reader_->dy(), Lz = nz * reader_->dz();

    const std::vector<double>& field = fieldArray(field_);

    // i-slowest/k-fastest, same convention as Fields3D everywhere else
    // (core/grid_index.hpp's idx3, unpadded variant -- see
    // pipeline/src/run_3d.cpp's detail::extract_preview_slice for the
    // exact same formula used there).
    auto idx = [ny, nz](int i, int j, int k) {
        return static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
             + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(k);
    };

    int texW = 0, texH = 0;
    std::vector<float> corners; // 4 verts * (x,y,z,u,v)
    std::vector<float> texel;   // texW*texH, row-major, row 0 = v=0

    auto extract = [&](int width, int height, auto sample) {
        texW = width;
        texH = height;
        texel.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                texel[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) + static_cast<std::size_t>(col)]
                    = static_cast<float>(sample(col, row));
            }
        }
    };

    if (axis_ == Axis::X) {
        int i = std::clamp(static_cast<int>(std::lround(slicePosition_ * (nx - 1))), 0, nx - 1);
        double x = i * reader_->dx();
        extract(nz, ny, [&](int k, int j) { return field[idx(i, j, k)]; }); // u<->k(nz), v<->j(ny)
        corners = {
            static_cast<float>(x), 0.0f, 0.0f, 0.0f, 0.0f,
            static_cast<float>(x), 0.0f, static_cast<float>(Lz), 1.0f, 0.0f,
            static_cast<float>(x), static_cast<float>(Ly), static_cast<float>(Lz), 1.0f, 1.0f,
            static_cast<float>(x), static_cast<float>(Ly), 0.0f, 0.0f, 1.0f,
        };
    } else if (axis_ == Axis::Y) {
        int j = std::clamp(static_cast<int>(std::lround(slicePosition_ * (ny - 1))), 0, ny - 1);
        double y = j * reader_->dy();
        extract(nx, nz, [&](int i, int k) { return field[idx(i, j, k)]; }); // u<->i(nx), v<->k(nz)
        corners = {
            0.0f, static_cast<float>(y), 0.0f, 0.0f, 0.0f,
            static_cast<float>(Lx), static_cast<float>(y), 0.0f, 1.0f, 0.0f,
            static_cast<float>(Lx), static_cast<float>(y), static_cast<float>(Lz), 1.0f, 1.0f,
            0.0f, static_cast<float>(y), static_cast<float>(Lz), 0.0f, 1.0f,
        };
    } else {
        int k = std::clamp(static_cast<int>(std::lround(slicePosition_ * (nz - 1))), 0, nz - 1);
        double z = k * reader_->dz();
        extract(nx, ny, [&](int i, int j) { return field[idx(i, j, k)]; }); // u<->i(nx), v<->j(ny)
        corners = {
            0.0f, 0.0f, static_cast<float>(z), 0.0f, 0.0f,
            static_cast<float>(Lx), 0.0f, static_cast<float>(z), 1.0f, 0.0f,
            static_cast<float>(Lx), static_cast<float>(Ly), static_cast<float>(z), 1.0f, 1.0f,
            0.0f, static_cast<float>(Ly), static_cast<float>(z), 0.0f, 1.0f,
        };
    }

    float vmin = texel.empty() ? 0.0f : texel.front();
    float vmax = vmin;
    for (float v : texel) {
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
    }

    sliceVao_.bind();
    sliceVbo_.bind();
    sliceVbo_.allocate(corners.data(), static_cast<int>(corners.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    sliceVbo_.release();
    sliceVao_.release();

    glBindTexture(GL_TEXTURE_2D, sliceTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, texW, texH, 0, GL_RED, GL_FLOAT, texel.data());
    sliceProgram_->bind();
    sliceProgram_->setUniformValue("uValueRange", QVector2D(vmin, vmax));
    sliceProgram_->release();

    haveSlice_ = true;
    emit valueRangeChanged(vmin, vmax);
}

bool ResultsViewerWidget::isSolidCell(int i, int j, int k) const {
    if (!reader_) return false;
    int nx = reader_->nx(), ny = reader_->ny(), nz = reader_->nz();
    if (i < 0 || i >= nx || j < 0 || j >= ny || k < 0 || k >= nz) return false;
    std::size_t idx = static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
                     + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(k);
    return idx < currentFrame_.obstacle.size() && currentFrame_.obstacle[idx] != 0.0f;
}

const std::vector<double>& ResultsViewerWidget::fieldArray(ScalarField field) const {
    switch (field) {
        case ScalarField::VelocityMagnitude: return currentFrame_.velocity_magnitude;
        case ScalarField::Pressure: return currentFrame_.pressure;
        case ScalarField::VelocityU: return currentFrame_.velocity_u;
        case ScalarField::VelocityV: return currentFrame_.velocity_v;
        case ScalarField::VelocityW: return currentFrame_.velocity_w;
    }
    return currentFrame_.velocity_magnitude;
}

double ResultsViewerWidget::sampleScalar(const std::vector<double>& field, double x, double y, double z) const {
    if (!haveFrame_ || !reader_) return 0.0;
    int nx = reader_->nx(), ny = reader_->ny(), nz = reader_->nz();
    double dx = reader_->dx(), dy = reader_->dy(), dz = reader_->dz();

    // Same no-cell-center-offset convention rebuildSlice() already uses
    // (texel col/row map directly to [0,Lx]/[0,Ly]/[0,Lz] without a
    // half-cell shift) -- consistent within this widget, good enough for
    // visualization, not meant for quantitative sampling.
    double fi = x / dx, fj = y / dy, fk = z / dz;
    int i0 = static_cast<int>(std::floor(fi));
    int j0 = static_cast<int>(std::floor(fj));
    int k0 = static_cast<int>(std::floor(fk));
    double tx = fi - i0, ty = fj - j0, tz = fk - k0;

    auto clampi = [](int v, int n) { return std::clamp(v, 0, n - 1); };
    auto at = [&](int i, int j, int k) {
        i = clampi(i, nx);
        j = clampi(j, ny);
        k = clampi(k, nz);
        std::size_t idx = static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
                         + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(k);
        return idx < field.size() ? field[idx] : 0.0;
    };
    double c00 = at(i0, j0, k0) * (1 - tx) + at(i0 + 1, j0, k0) * tx;
    double c01 = at(i0, j0, k0 + 1) * (1 - tx) + at(i0 + 1, j0, k0 + 1) * tx;
    double c10 = at(i0, j0 + 1, k0) * (1 - tx) + at(i0 + 1, j0 + 1, k0) * tx;
    double c11 = at(i0, j0 + 1, k0 + 1) * (1 - tx) + at(i0 + 1, j0 + 1, k0 + 1) * tx;
    double c0 = c00 * (1 - ty) + c10 * ty;
    double c1 = c01 * (1 - ty) + c11 * ty;
    return c0 * (1 - tz) + c1 * tz;
}

QVector3D ResultsViewerWidget::sampleVelocity(double x, double y, double z) const {
    if (!haveFrame_ || !reader_) return {};
    return QVector3D(static_cast<float>(sampleScalar(currentFrame_.velocity_u, x, y, z)),
                      static_cast<float>(sampleScalar(currentFrame_.velocity_v, x, y, z)),
                      static_cast<float>(sampleScalar(currentFrame_.velocity_w, x, y, z)));
}

std::vector<QVector3D> ResultsViewerWidget::traceStreamline(QVector3D start) const {
    std::vector<QVector3D> pts;
    if (!reader_) return pts;
    double Lx = reader_->nx() * reader_->dx(), Ly = reader_->ny() * reader_->dy(), Lz = reader_->nz() * reader_->dz();
    double dx = reader_->dx(), dy = reader_->dy(), dz = reader_->dz();
    float h = static_cast<float>(0.5 * std::min({dx, dy, dz})); // fixed arc-length step, not a time step
    constexpr int kMaxSteps = 800;
    constexpr float kMinSpeed = 1e-6f;

    QVector3D p = start;
    for (int step = 0; step < kMaxSteps; ++step) {
        if (p.x() < 0 || p.x() > Lx || p.y() < 0 || p.y() > Ly || p.z() < 0 || p.z() > Lz) break;
        int i = static_cast<int>(p.x() / dx), j = static_cast<int>(p.y() / dy), k = static_cast<int>(p.z() / dz);
        if (isSolidCell(i, j, k)) break;
        pts.push_back(p);

        // Classic RK4 stages for an arc-length-parameterized curve: each
        // stage direction is normalized to a unit vector *before* being
        // used to place the next sample point, so every sample offset is
        // exactly h/2 or h regardless of local speed. (An earlier version
        // used the raw, un-normalized velocity for those offsets, which is
        // dimensionally a length*speed, not a length -- at typical solver
        // velocity magnitudes that put p2/p3/p4 many step-lengths away from
        // p, so RK4's curvature-sampling benefit was lost and traces came
        // out visibly straighter than the actual flow.) The final blended
        // direction is likewise a step of fixed length h -- keeps step
        // count bounded regardless of local speed (near-stagnant
        // recirculation zones would otherwise take forever to advance).
        QVector3D v1 = sampleVelocity(p.x(), p.y(), p.z());
        if (v1.length() < kMinSpeed) break;
        QVector3D k1 = v1.normalized();
        QVector3D p2 = p + k1 * (h * 0.5f);
        QVector3D v2 = sampleVelocity(p2.x(), p2.y(), p2.z());
        if (v2.length() < kMinSpeed) break;
        QVector3D k2 = v2.normalized();
        QVector3D p3 = p + k2 * (h * 0.5f);
        QVector3D v3 = sampleVelocity(p3.x(), p3.y(), p3.z());
        if (v3.length() < kMinSpeed) break;
        QVector3D k3 = v3.normalized();
        QVector3D p4 = p + k3 * h;
        QVector3D v4 = sampleVelocity(p4.x(), p4.y(), p4.z());
        if (v4.length() < kMinSpeed) break;
        QVector3D k4 = v4.normalized();
        QVector3D blended = (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (1.0f / 6.0f);
        if (blended.length() < kMinSpeed) break; // near-cancelling directions -- a singular point in the direction field
        p = p + blended.normalized() * h;
    }
    return pts;
}

void ResultsViewerWidget::rebuildStreamlines() {
    streamlinesDirty_ = false;
    streamlineRunLengths_.clear();
    if (!haveFrame_ || !reader_ || !showStreamlines_) return;

    double Lx = reader_->nx() * reader_->dx(), Ly = reader_->ny() * reader_->dy(), Lz = reader_->nz() * reader_->dz();
    int kSeedGrid = vectorDensity_; // density x density seeds across the inflow face

    // Bias the seed grid toward the object's own cross-section (plus a
    // margin) rather than spreading it uniformly across the whole inflow
    // face: in a typical external-flow domain the object is a small
    // fraction of that face, so uniform seeding mostly produced lines that
    // never passed close enough to the object to visibly deflect. The same
    // margined box is reused below to drop any traced line that still
    // never actually enters it -- a dense seed grid otherwise leaves a lot
    // of "extra" streamlines that run straight through empty space well
    // off to the side of the object.
    double yLo = 0.0, yHi = Ly, zLo = 0.0, zHi = Lz;
    bool haveProximityBox = false;
    QVector3D proxMin, proxMax;
    if (haveMesh_) {
        auto b = mesh_.bounds();
        double marginX = std::max(0.15 * (b.max.x - b.min.x), 0.05 * Lx);
        double marginY = std::max(0.15 * (b.max.y - b.min.y), 0.05 * Ly);
        double marginZ = std::max(0.15 * (b.max.z - b.min.z), 0.05 * Lz);
        double loY = std::clamp(b.min.y - marginY, 0.0, Ly);
        double hiY = std::clamp(b.max.y + marginY, 0.0, Ly);
        double loZ = std::clamp(b.min.z - marginZ, 0.0, Lz);
        double hiZ = std::clamp(b.max.z + marginZ, 0.0, Lz);
        if (hiY > loY) { yLo = loY; yHi = hiY; }
        if (hiZ > loZ) { zLo = loZ; zHi = hiZ; }

        proxMin = QVector3D(static_cast<float>(b.min.x - marginX), static_cast<float>(b.min.y - marginY),
                             static_cast<float>(b.min.z - marginZ));
        proxMax = QVector3D(static_cast<float>(b.max.x + marginX), static_cast<float>(b.max.y + marginY),
                             static_cast<float>(b.max.z + marginZ));
        haveProximityBox = true;
    }
    auto passesNearObject = [&](const std::vector<QVector3D>& line) {
        if (!haveProximityBox) return true; // no mesh bounds available -- keep everything
        for (const auto& p : line) {
            if (p.x() >= proxMin.x() && p.x() <= proxMax.x() && p.y() >= proxMin.y() && p.y() <= proxMax.y() &&
                p.z() >= proxMin.z() && p.z() <= proxMax.z()) {
                return true;
            }
        }
        return false;
    };

    std::vector<std::vector<QVector3D>> lines;
    for (int a = 0; a < kSeedGrid; ++a) {
        for (int b = 0; b < kSeedGrid; ++b) {
            float y = static_cast<float>(yLo + (a + 0.5) / kSeedGrid * (yHi - yLo));
            float z = static_cast<float>(zLo + (b + 0.5) / kSeedGrid * (zHi - zLo));
            QVector3D seed(static_cast<float>(0.02 * Lx), y, z);
            int i = static_cast<int>(seed.x() / reader_->dx());
            int j = static_cast<int>(seed.y() / reader_->dy());
            int k = static_cast<int>(seed.z() / reader_->dz());
            if (isSolidCell(i, j, k)) continue;
            auto line = traceStreamline(seed);
            if (line.size() >= 2 && passesNearObject(line)) lines.push_back(std::move(line));
        }
    }

    // Color by local speed, normalized against this streamline set's own
    // min/max -- independent of the slice's field/range (streamlines are
    // always a velocity-direction visualization, whatever scalar the
    // slice happens to be showing).
    float vmin = std::numeric_limits<float>::infinity(), vmax = -std::numeric_limits<float>::infinity();
    for (const auto& line : lines) {
        for (const auto& p : line) {
            float speed = sampleVelocity(p.x(), p.y(), p.z()).length();
            vmin = std::min(vmin, speed);
            vmax = std::max(vmax, speed);
        }
    }
    float range = (vmax > vmin) ? (vmax - vmin) : 1.0f;

    std::vector<float> vertexData;
    for (const auto& line : lines) {
        for (const auto& p : line) {
            float speed = sampleVelocity(p.x(), p.y(), p.z()).length();
            QColor c = flow_colormap_sample((speed - vmin) / range);
            vertexData.insert(vertexData.end(), {p.x(), p.y(), p.z(), static_cast<float>(c.redF()),
                                                  static_cast<float>(c.greenF()), static_cast<float>(c.blueF())});
        }
        streamlineRunLengths_.push_back(static_cast<int>(line.size()));
    }

    streamlineVao_.bind();
    streamlineVbo_.bind();
    streamlineVbo_.allocate(vertexData.data(), static_cast<int>(vertexData.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    streamlineVbo_.release();
    streamlineVao_.release();
}

void ResultsViewerWidget::rebuildArrows() {
    arrowsDirty_ = false;
    arrowVertexCount_ = 0;
    if (!haveFrame_ || !reader_ || !showArrows_) return;

    int nx = reader_->nx(), ny = reader_->ny(), nz = reader_->nz();
    double dx = reader_->dx(), dy = reader_->dy(), dz = reader_->dz();
    double Lx = nx * dx, Ly = ny * dy, Lz = nz * dz;
    int kGrid = vectorDensity_; // subsample the slice plane to a density x density arrow grid

    // In-plane footprint the density x density grid spreads across depends
    // on which two axes the current slice varies over -- e.g. an X slice's
    // grid spans Y and Z, not X. Max arrow length is a fraction of the
    // spacing *between adjacent samples* (not the solver's own cell size),
    // so raising density automatically shrinks arrows to match the denser
    // grid instead of making them overlap.
    double footprintU = 0.0, footprintV = 0.0;
    switch (axis_) {
        case Axis::X: footprintU = Lz; footprintV = Ly; break;
        case Axis::Y: footprintU = Lx; footprintV = Lz; break;
        case Axis::Z: footprintU = Lx; footprintV = Ly; break;
    }
    float maxArrowLen = static_cast<float>(0.7 * std::min(footprintU, footprintV) / kGrid);

    struct Sample {
        QVector3D pos;
        QVector3D vel;
    };
    std::vector<Sample> samples;
    auto collect = [&](auto makePos) {
        for (int a = 0; a < kGrid; ++a) {
            for (int b = 0; b < kGrid; ++b) {
                QVector3D pos = makePos((a + 0.5) / kGrid, (b + 0.5) / kGrid);
                int i = static_cast<int>(pos.x() / dx), j = static_cast<int>(pos.y() / dy), k = static_cast<int>(pos.z() / dz);
                if (isSolidCell(i, j, k)) continue;
                samples.push_back({pos, sampleVelocity(pos.x(), pos.y(), pos.z())});
            }
        }
    };
    if (axis_ == Axis::X) {
        float x = static_cast<float>(std::clamp(std::lround(slicePosition_ * (nx - 1)), 0L, static_cast<long>(nx - 1)) * dx);
        collect([&](double u, double v) { return QVector3D(x, static_cast<float>(v * Ly), static_cast<float>(u * Lz)); });
    } else if (axis_ == Axis::Y) {
        float y = static_cast<float>(std::clamp(std::lround(slicePosition_ * (ny - 1)), 0L, static_cast<long>(ny - 1)) * dy);
        collect([&](double u, double v) { return QVector3D(static_cast<float>(u * Lx), y, static_cast<float>(v * Lz)); });
    } else {
        float z = static_cast<float>(std::clamp(std::lround(slicePosition_ * (nz - 1)), 0L, static_cast<long>(nz - 1)) * dz);
        collect([&](double u, double v) { return QVector3D(static_cast<float>(u * Lx), static_cast<float>(v * Ly), z); });
    }

    float vmin = std::numeric_limits<float>::infinity(), vmax = -std::numeric_limits<float>::infinity();
    for (const auto& s : samples) {
        vmin = std::min(vmin, s.vel.length());
        vmax = std::max(vmax, s.vel.length());
    }
    float range = (vmax > vmin) ? (vmax - vmin) : 1.0f;

    std::vector<float> vertexData;
    for (const auto& s : samples) {
        float speed = s.vel.length();
        if (speed < 1e-6f) continue;
        QVector3D dir = s.vel / speed;
        float len = std::min(speed / vmax, 1.0f) * maxArrowLen;
        QColor c = flow_colormap_sample((speed - vmin) / range);
        QVector3D color(static_cast<float>(c.redF()), static_cast<float>(c.greenF()), static_cast<float>(c.blueF()));

        QVector3D tip = s.pos + dir * len;
        QVector3D perp = QVector3D::crossProduct(dir, QVector3D(0, 1, 0));
        if (perp.lengthSquared() < 1e-6f) perp = QVector3D::crossProduct(dir, QVector3D(1, 0, 0));
        perp.normalize();
        float headSize = len * 0.35f;
        QVector3D back = tip - dir * headSize;
        auto pushVert = [&](const QVector3D& p) {
            vertexData.insert(vertexData.end(), {p.x(), p.y(), p.z(), color.x(), color.y(), color.z()});
        };
        pushVert(s.pos);
        pushVert(tip); // shaft
        pushVert(tip);
        pushVert(back + perp * headSize * 0.5f); // head leg 1
        pushVert(tip);
        pushVert(back - perp * headSize * 0.5f); // head leg 2
    }
    arrowVertexCount_ = static_cast<int>(vertexData.size() / 6);

    arrowVao_.bind();
    arrowVbo_.bind();
    arrowVbo_.allocate(vertexData.data(), static_cast<int>(vertexData.size() * sizeof(float)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    arrowVbo_.release();
    arrowVao_.release();
}

void ResultsViewerWidget::rebuildVortexVolume() {
    volumeDirty_ = false;
    haveVolume_ = false;
    if (!haveFrame_ || !reader_ || !showVortexVolume_) return;

    int nx = reader_->nx(), ny = reader_->ny(), nz = reader_->nz();
    double dx = reader_->dx(), dy = reader_->dy(), dz = reader_->dz();
    const auto& u = currentFrame_.velocity_u;
    const auto& v = currentFrame_.velocity_v;
    const auto& w = currentFrame_.velocity_w;
    if (static_cast<int>(u.size()) != nx * ny * nz) return;

    auto clampi = [](int val, int n) { return std::clamp(val, 0, n - 1); };
    auto at = [&](const std::vector<double>& f, int i, int j, int k) {
        i = clampi(i, nx);
        j = clampi(j, ny);
        k = clampi(k, nz);
        return f[static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
                 + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(k)];
    };

    // Q-criterion: 0.5*(||rotation-rate||^2 - ||strain-rate||^2) from the
    // velocity gradient tensor -- positive where rotation dominates strain,
    // the standard scalar for identifying vortex cores (unlike raw
    // vorticity magnitude, which also lights up ordinary shear layers,
    // e.g. right at the object's own boundary layer).
    std::vector<float> qField(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz),
                               0.0f);
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            for (int k = 0; k < nz; ++k) {
                double dudx = (at(u, i + 1, j, k) - at(u, i - 1, j, k)) / (2.0 * dx);
                double dudy = (at(u, i, j + 1, k) - at(u, i, j - 1, k)) / (2.0 * dy);
                double dudz = (at(u, i, j, k + 1) - at(u, i, j, k - 1)) / (2.0 * dz);
                double dvdx = (at(v, i + 1, j, k) - at(v, i - 1, j, k)) / (2.0 * dx);
                double dvdy = (at(v, i, j + 1, k) - at(v, i, j - 1, k)) / (2.0 * dy);
                double dvdz = (at(v, i, j, k + 1) - at(v, i, j, k - 1)) / (2.0 * dz);
                double dwdx = (at(w, i + 1, j, k) - at(w, i - 1, j, k)) / (2.0 * dx);
                double dwdy = (at(w, i, j + 1, k) - at(w, i, j - 1, k)) / (2.0 * dy);
                double dwdz = (at(w, i, j, k + 1) - at(w, i, j, k - 1)) / (2.0 * dz);

                double Sxy = 0.5 * (dudy + dvdx), Sxz = 0.5 * (dudz + dwdx), Syz = 0.5 * (dvdz + dwdy);
                double Oxy = 0.5 * (dudy - dvdx), Oxz = 0.5 * (dudz - dwdx), Oyz = 0.5 * (dvdz - dwdy);
                double S2 = dudx * dudx + dvdy * dvdy + dwdz * dwdz + 2.0 * (Sxy * Sxy + Sxz * Sxz + Syz * Syz);
                double O2 = 2.0 * (Oxy * Oxy + Oxz * Oxz + Oyz * Oyz);
                double q = 0.5 * (O2 - S2);

                std::size_t flat = static_cast<std::size_t>(i) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz)
                                  + static_cast<std::size_t>(j) * static_cast<std::size_t>(nz) + static_cast<std::size_t>(k);
                qField[flat] = static_cast<float>(std::max(q, 0.0));
            }
        }
    }

    // Zero it inside/at the solid object so the body itself never reads as
    // a false "vortex" -- boundary-layer velocity gradients right at a
    // no-slip wall are large and rotation-dominated by construction, which
    // would otherwise paint the object's own surface into the volume.
    for (std::size_t idx = 0; idx < qField.size() && idx < currentFrame_.obstacle.size(); ++idx) {
        if (currentFrame_.obstacle[idx] != 0.0f) qField[idx] = 0.0f;
    }

    // Normalize against the 98th percentile of the positive values, not the
    // single global max: one extreme outlier cell (e.g. right at a
    // stagnation point or a sharp mesh feature) squashes every genuine
    // wake-vortex structure down near the low end of the 0..1 range,
    // making them invisible to the shader's transfer function even though
    // they're the actual vortex cores worth showing.
    std::vector<float> positives;
    positives.reserve(qField.size() / 16);
    for (float q : qField) {
        if (q > 0.0f) positives.push_back(q);
    }
    float qref = 1e-12f;
    if (!positives.empty()) {
        std::sort(positives.begin(), positives.end());
        std::size_t idx = static_cast<std::size_t>(0.98 * static_cast<double>(positives.size() - 1));
        qref = std::max(positives[idx], 1e-12f);
    }
    for (float& q : qField) q = std::min(q / qref, 1.0f); // values above the percentile just clamp to 1

    glBindTexture(GL_TEXTURE_3D, volumeTexture_);
    // GL's (width,height,depth) order must match the data's own fastest-to-
    // slowest axis order: qField is i-slowest/k-fastest (nx*ny*nz layout,
    // same convention as Fields3D everywhere else in this app), so
    // width=nz (fastest), height=ny, depth=nx (slowest) -- NOT (nx,ny,nz).
    // The fragment shader's texture lookup mirrors this with (s,t,r) =
    // (zFrac, yFrac, xFrac).
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R32F, nz, ny, nx, 0, GL_RED, GL_FLOAT, qField.data());

    double Lx = nx * dx, Ly = ny * dy, Lz = nz * dz;
    // clang-format off
    float box[] = {
        // -X                                          +X
        0,0,0,  0,(float)Ly,0,  0,(float)Ly,(float)Lz,  0,0,0,  0,(float)Ly,(float)Lz,  0,0,(float)Lz,
        (float)Lx,0,0,  (float)Lx,0,(float)Lz,  (float)Lx,(float)Ly,(float)Lz,  (float)Lx,0,0,  (float)Lx,(float)Ly,(float)Lz,  (float)Lx,(float)Ly,0,
        // -Y                                          +Y
        0,0,0,  (float)Lx,0,0,  (float)Lx,0,(float)Lz,  0,0,0,  (float)Lx,0,(float)Lz,  0,0,(float)Lz,
        0,(float)Ly,0,  0,(float)Ly,(float)Lz,  (float)Lx,(float)Ly,(float)Lz,  0,(float)Ly,0,  (float)Lx,(float)Ly,(float)Lz,  (float)Lx,(float)Ly,0,
        // -Z                                          +Z
        0,0,0,  0,(float)Ly,0,  (float)Lx,(float)Ly,0,  0,0,0,  (float)Lx,(float)Ly,0,  (float)Lx,0,0,
        0,0,(float)Lz,  (float)Lx,0,(float)Lz,  (float)Lx,(float)Ly,(float)Lz,  0,0,(float)Lz,  (float)Lx,(float)Ly,(float)Lz,  0,(float)Ly,(float)Lz,
    };
    // clang-format on

    volumeBoxVao_.bind();
    volumeBoxVbo_.bind();
    volumeBoxVbo_.allocate(box, static_cast<int>(sizeof(box)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void*>(0));
    volumeBoxVbo_.release();
    volumeBoxVao_.release();

    haveVolume_ = true;
}

QVector3D ResultsViewerWidget::eyePosition() const {
    float yawRad = yaw_ * kDegToRad;
    float pitchRad = pitch_ * kDegToRad;
    return QVector3D(center_.x() + distance_ * std::cos(pitchRad) * std::sin(yawRad),
                      center_.y() + distance_ * std::sin(pitchRad),
                      center_.z() + distance_ * std::cos(pitchRad) * std::cos(yawRad));
}

QMatrix4x4 ResultsViewerWidget::viewMatrix() const {
    QVector3D eye = eyePosition();
    QMatrix4x4 view;
    view.lookAt(eye, center_, QVector3D(0, 1, 0));
    return view;
}

QMatrix4x4 ResultsViewerWidget::projectionMatrix() const {
    QMatrix4x4 proj;
    float aspect = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    proj.perspective(45.0f, aspect, std::max(0.01f, radius_ * 0.01f), std::max(radius_ * 100.0f, 10.0f));
    return proj;
}

void ResultsViewerWidget::paintGL() {
    QColor clearColor(theme_.plot_bg);
    glClearColor(static_cast<float>(clearColor.redF()), static_cast<float>(clearColor.greenF()),
                 static_cast<float>(clearColor.blueF()), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (meshDirty_ && haveMesh_) rebuildMeshGeometry();
    if (sliceDirty_) rebuildSlice();
    if (streamlinesDirty_) rebuildStreamlines();
    if (arrowsDirty_) rebuildArrows();
    if (volumeDirty_) rebuildVortexVolume();

    QMatrix4x4 mvp = projectionMatrix() * viewMatrix();

    if (haveMesh_) {
        meshProgram_->bind();
        meshProgram_->setUniformValue("uMvp", mvp);
        meshProgram_->setUniformValue("uLightDir", QVector3D(-0.4f, -1.0f, -0.3f));
        meshVao_.bind();
        glDrawArrays(GL_TRIANGLES, 0, meshVertexCount_);
        meshVao_.release();
        meshProgram_->release();
    }

    if (haveSlice_) {
        sliceProgram_->bind();
        sliceProgram_->setUniformValue("uMvp", mvp);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, sliceTexture_);
        sliceProgram_->setUniformValue("uField", 0);
        sliceVao_.bind();
        glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        sliceVao_.release();
        sliceProgram_->release();
    }

    if ((showStreamlines_ && !streamlineRunLengths_.empty()) || (showArrows_ && arrowVertexCount_ > 0)) {
        vectorProgram_->bind();
        vectorProgram_->setUniformValue("uMvp", mvp);

        if (showStreamlines_ && !streamlineRunLengths_.empty()) {
            // Wider than the 1px default so a dense set reads as a flowing
            // ribbon sheet rather than a thin wireframe tangle -- how wide
            // a line actually renders past 1px is driver-dependent in a
            // core profile, so this is a best-effort bump, not guaranteed.
            glLineWidth(2.0f);
            streamlineVao_.bind();
            int offset = 0;
            for (int count : streamlineRunLengths_) {
                glDrawArrays(GL_LINE_STRIP, offset, count);
                offset += count;
            }
            streamlineVao_.release();
            glLineWidth(1.0f);
        }
        if (showArrows_ && arrowVertexCount_ > 0) {
            arrowVao_.bind();
            glDrawArrays(GL_LINES, 0, arrowVertexCount_);
            arrowVao_.release();
        }

        vectorProgram_->release();
    }

    if (showVortexVolume_ && haveVolume_ && reader_) {
        volumeProgram_->bind();
        volumeProgram_->setUniformValue("uMvp", mvp);
        volumeProgram_->setUniformValue("uCameraPos", eyePosition());
        volumeProgram_->setUniformValue("uBoxMin", QVector3D(0.0f, 0.0f, 0.0f));
        volumeProgram_->setUniformValue("uBoxMax",
                                         QVector3D(static_cast<float>(reader_->nx() * reader_->dx()),
                                                    static_cast<float>(reader_->ny() * reader_->dy()),
                                                    static_cast<float>(reader_->nz() * reader_->dz())));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, volumeTexture_);
        volumeProgram_->setUniformValue("uVolume", 0);

        // Premultiplied-alpha "over" blending, matching how the fragment
        // shader accumulates accum.rgb -- depth test stays on (so the
        // volume is naturally clipped by nearer opaque geometry like the
        // mesh at the box's own boundary), but depth write is off so this
        // translucent pass doesn't occlude anything drawn after it.
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        volumeBoxVao_.bind();
        glDrawArrays(GL_TRIANGLES, 0, 36);
        volumeBoxVao_.release();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        volumeProgram_->release();
    }
}

void ResultsViewerWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = true;
        lastMousePos_ = event->pos();
    }
}

void ResultsViewerWidget::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_ && (event->buttons() & Qt::LeftButton)) {
        QPoint delta = event->pos() - lastMousePos_;
        lastMousePos_ = event->pos();
        yaw_ += static_cast<float>(delta.x()) * 0.4f;
        pitch_ = std::clamp(pitch_ - static_cast<float>(delta.y()) * 0.4f, -89.0f, 89.0f);
        update();
    }
}

void ResultsViewerWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) dragging_ = false;
}

void ResultsViewerWidget::wheelEvent(QWheelEvent* event) {
    float factor = event->angleDelta().y() > 0 ? 0.9f : 1.1f;
    distance_ = std::clamp(distance_ * factor, radius_ * 0.5f, radius_ * 20.0f);
    update();
    event->accept();
}
