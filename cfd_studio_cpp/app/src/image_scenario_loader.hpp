#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <QImage>
#include <QString>

// Loads a PNG/JPG/WEBP file (PNG/JPG via Qt's own QImage; WEBP has no Qt
// plugin in this build, so it's decoded directly via libwebp -- see the
// .cpp) and rasterizes it into a solid/fluid mask for the 2D custom-image
// scenario.
namespace image_scenario {

// nullopt on decode failure (unreadable file, unsupported/corrupt format).
[[nodiscard]] std::optional<QImage> loadImage(const QString& path);

// Resizes `image` to nx*ny (non-uniform stretch, matching the grid the
// solver will actually run at -- simplest and most predictable for a user
// picking a resolution directly) and thresholds it to a boolean mask, 1 =
// solid. A pixel counts as solid when it's both mostly opaque (alpha >
// 50%) and darker than `threshold` (0..1 relative luminance) -- covers
// both a silhouette drawn on a transparent background and a dark shape on
// a light background, the two most natural ways to make an image like
// this. The returned mask is already in "display" orientation (row 0 =
// bottom of the image, matching Fields2D/PlotWidget's convention), ready
// for cfd::solvers::orient_mask_for_solver().
[[nodiscard]] std::vector<std::uint8_t> rasterizeToMask(const QImage& image, int nx, int ny, double threshold = 0.5);

} // namespace image_scenario
