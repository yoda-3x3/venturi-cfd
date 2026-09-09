#pragma once

#include <memory>
#include <vector>

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>

#include "io/results_cache_reader.hpp"
#include "mesh/mesh.hpp"
#include "theme.hpp"

// Renders the object mesh plus one movable, scalar-field-colored slice
// plane through a loaded results-cache frame -- Phase 1 ("walking
// skeleton") of the in-app results visualizer, so a completed run can be
// inspected without launching external ParaView. Same architectural
// pattern as MeshPreviewWidget (orbit camera, hand-rolled GLSL,
// construction-time Theme injection -- not live-updating across a theme
// switch, an accepted limitation matching how OrientationDialog already
// works): reuses its lit mesh shader unchanged and adds a new
// textured-quad shader for the slice.
//
// Inherits the full desktop 3.3 core function set (not just Qt's
// ES2-subset QOpenGLFunctions, used by every other GL widget in this
// app) because the vortex-volume ray marcher below needs glTexImage3D
// (3D textures aren't part of ES2) -- a strict superset, so every
// existing ES2-compatible call (glTexImage2D, glDrawArrays, ...) still
// works unchanged.
class ResultsViewerWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core {
    Q_OBJECT

public:
    enum class ScalarField { VelocityMagnitude, Pressure, VelocityU, VelocityV, VelocityW };
    enum class Axis { X, Y, Z };

    explicit ResultsViewerWidget(QWidget* parent = nullptr);
    ~ResultsViewerWidget() override;

    void setTheme(const Theme& theme);

    // Uploads the object mesh (unchanged across frames/fields/slices).
    // Resets the camera to frame its bounding box -- same contract as
    // MeshPreviewWidget::setMesh.
    void setMesh(const cfd::mesh::Mesh& mesh);

    // Points this widget at an already-opened results cache. The caller
    // is expected to have constructed `reader` itself (its constructor
    // can throw on a missing/corrupt cache) so a failure can be reported
    // before this widget is even shown, rather than surfaced from inside
    // a setter here. Defaults to frame 0.
    void setResultsCache(std::shared_ptr<cfd::io::ResultsCacheReader> reader);

    [[nodiscard]] int frameCount() const;
    [[nodiscard]] double frameTime(int index) const;

    void setFrame(int index);
    void setScalarField(ScalarField field);
    void setSliceAxis(Axis axis);
    void setSlicePosition(double t); // 0..1, normalized along the slice axis

    // Streamlines: RK4-traced 3D lines through the velocity field, seeded
    // from a grid on the inflow face (x=0) -- independent of the slice
    // plane, so they convey the actual 3D flow regardless of which slice
    // is shown. Velocity arrow glyphs: small 3D arrows sampled on the
    // *current* slice plane's grid (so they stay visually tied to what's
    // shown, rather than cluttering the whole volume). Both colored by
    // local velocity magnitude, normalized independently of each other and
    // of the slice's own field/range.
    void setShowStreamlines(bool show);
    void setShowVelocityArrows(bool show);

    // Per-axis grid resolution for both overlays: streamlines seed from a
    // density x density grid on the inflow face, arrows sample a
    // density x density grid on the current slice. One shared control
    // since both are "how many flow glyphs" from the user's point of view.
    void setVectorDensity(int density);

    // Volumetric ray-marched render of vortex cores: computes the
    // Q-criterion (rotation-rate-squared minus strain-rate-squared) at
    // every grid point from the velocity field's own gradients, keeps only
    // the rotation-dominated (Q>0) regions -- the standard scalar for
    // identifying vortex cores, not just generic velocity magnitude -- and
    // marches camera rays through the resulting 3D density field each
    // frame, accumulating a fire-toned color by alpha compositing.
    void setShowVortexVolume(bool show);

signals:
    // Fired whenever the displayed slice's value range changes (new frame,
    // field, axis, or position) -- drives ColorLegendWidget's bar.
    void valueRangeChanged(double vmin, double vmax);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void rebuildMeshGeometry();
    void rebuildSlice(); // re-extracts the CPU slice buffer, re-uploads the texture, recomputes the quad corners
    void rebuildStreamlines();
    void rebuildArrows();
    // Computes Q-criterion on the grid, uploads it as a 3D texture, and
    // rebuilds the domain-bounds proxy cube the fragment shader ray-marches
    // through.
    void rebuildVortexVolume();
    [[nodiscard]] QMatrix4x4 viewMatrix() const;
    [[nodiscard]] QMatrix4x4 projectionMatrix() const;
    [[nodiscard]] QVector3D eyePosition() const; // orbit camera's world/domain-space eye position
    // Trilinear-interpolated value of one field array at a point in
    // domain-space coordinates (same [0,Lx]x[0,Ly]x[0,Lz] frame as the
    // slice quads) -- shared by sampleVelocity (u/v/w) and the mesh
    // surface's own near-wall field sampling.
    [[nodiscard]] double sampleScalar(const std::vector<double>& field, double x, double y, double z) const;
    // Which of currentFrame_'s arrays a ScalarField selects -- shared by
    // rebuildSlice and rebuildMeshGeometry so both read the same field.
    [[nodiscard]] const std::vector<double>& fieldArray(ScalarField field) const;
    [[nodiscard]] QVector3D sampleVelocity(double x, double y, double z) const;
    [[nodiscard]] bool isSolidCell(int i, int j, int k) const;
    // One streamline's worth of RK4-traced points, domain-space, starting
    // at `start`; empty if it starts inside a solid cell.
    [[nodiscard]] std::vector<QVector3D> traceStreamline(QVector3D start) const;

    Theme theme_ = theme_by_key(kDefaultThemeKey);

    cfd::mesh::Mesh mesh_;
    bool haveMesh_ = false;
    bool meshDirty_ = false;
    QOpenGLBuffer meshVbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject meshVao_;
    int meshVertexCount_ = 0;
    std::unique_ptr<QOpenGLShaderProgram> meshProgram_;

    QOpenGLBuffer sliceVbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject sliceVao_;
    std::unique_ptr<QOpenGLShaderProgram> sliceProgram_;
    unsigned int sliceTexture_ = 0;
    bool sliceDirty_ = false;
    bool haveSlice_ = false;

    // Shared position+color line shader/pipeline for both streamlines
    // (drawn as one GL_LINE_STRIP run per line) and arrow glyphs (drawn as
    // one GL_LINES call, 3 segments/6 vertices per arrow: shaft + 2 head
    // legs) -- both are just colored line segments, no need for two
    // shaders.
    std::unique_ptr<QOpenGLShaderProgram> vectorProgram_;
    QOpenGLBuffer streamlineVbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject streamlineVao_;
    std::vector<int> streamlineRunLengths_; // vertex count per streamline, for per-line glDrawArrays calls
    bool showStreamlines_ = false;
    bool streamlinesDirty_ = false;
    QOpenGLBuffer arrowVbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject arrowVao_;
    int arrowVertexCount_ = 0;
    bool showArrows_ = false;
    bool arrowsDirty_ = false;
    // Denser default than the original walking-skeleton pass -- a sparse
    // grid mostly missed the object; a denser sheet of lines hugging its
    // silhouette reads much closer to typical CFD streamline renders.
    int vectorDensity_ = 18;

    // Vortex-core volume: a proxy cube spanning [0,Lx]x[0,Ly]x[0,Lz]
    // (position-only, matches the domain frame every other overlay
    // already uses) that the fragment shader ray-marches through,
    // sampling a 3D texture of the (normalized, Q>0-only) Q-criterion
    // field.
    std::unique_ptr<QOpenGLShaderProgram> volumeProgram_;
    QOpenGLBuffer volumeBoxVbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject volumeBoxVao_;
    unsigned int volumeTexture_ = 0;
    bool showVortexVolume_ = false;
    bool volumeDirty_ = false;
    bool haveVolume_ = false;

    std::shared_ptr<cfd::io::ResultsCacheReader> reader_;
    cfd::io::ResultsFrame currentFrame_;
    bool haveFrame_ = false;
    int frameIndex_ = 0;
    ScalarField field_ = ScalarField::VelocityMagnitude;
    Axis axis_ = Axis::Y;
    double slicePosition_ = 0.5;

    QVector3D center_{0, 0, 0};
    float radius_ = 1.0f;
    float yaw_ = -45.0f, pitch_ = 25.0f, distance_ = 3.0f;
    QPoint lastMousePos_;
    bool dragging_ = false;
};
