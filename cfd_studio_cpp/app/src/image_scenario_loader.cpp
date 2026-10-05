#include "image_scenario_loader.hpp"

#include <cstring>

#include <QFile>
#include <QFileInfo>

#include <webp/decode.h>

namespace image_scenario {

namespace {
// QImage has no WEBP plugin in this Qt build (see cfd_studio_cpp/BUILD.md
// for the full toolchain list -- no qtimageformats webp plugin was ever
// deployed), so a .webp file is decoded straight from libwebp's simple
// API into an RGBA buffer, then copied into a QImage. WebPDecodeRGBA
// returns a malloc'd buffer that must be released with WebPFree, not
// delete/free directly by name (matches libwebp's own documented usage).
std::optional<QImage> loadWebp(const QByteArray& bytes) {
    int width = 0, height = 0;
    std::uint8_t* rgba = WebPDecodeRGBA(reinterpret_cast<const std::uint8_t*>(bytes.constData()),
                                        static_cast<std::size_t>(bytes.size()), &width, &height);
    if (!rgba || width <= 0 || height <= 0) return std::nullopt;

    QImage image(width, height, QImage::Format_RGBA8888);
    std::size_t rowBytes = static_cast<std::size_t>(width) * 4;
    for (int y = 0; y < height; ++y) {
        std::memcpy(image.scanLine(y), rgba + static_cast<std::size_t>(y) * rowBytes, rowBytes);
    }
    WebPFree(rgba);
    return image;
}
} // namespace

std::optional<QImage> loadImage(const QString& path) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) return std::nullopt;
    QByteArray bytes = file.readAll();
    file.close();

    QString suffix = QFileInfo(path).suffix().toLower();
    if (suffix == "webp") return loadWebp(bytes);

    QImage image;
    if (!image.loadFromData(bytes)) return std::nullopt;
    return image;
}

std::vector<std::uint8_t> rasterizeToMask(const QImage& image, int nx, int ny, double threshold) {
    QImage scaled = image.convertToFormat(QImage::Format_ARGB32)
                        .scaled(nx, ny, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);

    std::vector<std::uint8_t> mask(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0);
    for (int row = 0; row < ny; ++row) {
        // Row 0 of the image is the TOP; row 0 of the mask is the BOTTOM
        // (physical y=0), matching Fields2D/idx2/PlotWidget's convention
        // throughout the rest of the app.
        int j = ny - 1 - row;
        const QRgb* line = reinterpret_cast<const QRgb*>(scaled.constScanLine(row));
        for (int i = 0; i < nx; ++i) {
            QColor c(line[i]);
            double alpha = c.alphaF();
            double luminance = 0.2126 * c.redF() + 0.7152 * c.greenF() + 0.0722 * c.blueF();
            bool solid = alpha > 0.5 && luminance < threshold;
            mask[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] =
                solid ? 1 : 0;
        }
    }
    return mask;
}

} // namespace image_scenario
