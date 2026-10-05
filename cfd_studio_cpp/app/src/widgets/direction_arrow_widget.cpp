#include "direction_arrow_widget.hpp"

#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>

using cfd::solvers::ImageFlowDirection;

namespace {
constexpr int kMinDragPixels = 12; // shorter than this reads as a click, not a drag -- direction stays unchanged

ImageFlowDirection snapToCardinal(QPoint delta) {
    // Qt widget coordinates grow downward; flip y so "up" on screen (which
    // is also "up" in the uploaded image and the resulting flow field,
    // per rasterizeToMask's own row flip) maps to +90 degrees like a
    // normal math angle, not -90.
    double angleDeg = std::atan2(static_cast<double>(-delta.y()), static_cast<double>(delta.x())) * 180.0 / M_PI;
    if (angleDeg > -45.0 && angleDeg <= 45.0) return ImageFlowDirection::Right;
    if (angleDeg > 45.0 && angleDeg <= 135.0) return ImageFlowDirection::Up;
    if (angleDeg > -135.0 && angleDeg <= -45.0) return ImageFlowDirection::Down;
    return ImageFlowDirection::Left;
}
} // namespace

DirectionArrowWidget::DirectionArrowWidget(QWidget* parent) : QWidget(parent) {
    setMinimumHeight(160);
    setMouseTracking(false);
    setCursor(Qt::CrossCursor);
}

void DirectionArrowWidget::setImage(const QImage& image) {
    image_ = image;
    update();
}

void DirectionArrowWidget::setTheme(const Theme& theme) {
    theme_ = theme;
    update();
}

QRect DirectionArrowWidget::imageRect() const {
    if (image_.isNull()) return rect().adjusted(4, 4, -4, -4);
    QSize fitted = image_.size().scaled(rect().size(), Qt::KeepAspectRatio);
    QRect r(QPoint(0, 0), fitted);
    r.moveCenter(rect().center());
    return r;
}

void DirectionArrowWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(theme_.input_bg.isEmpty() ? "#10161f" : theme_.input_bg));

    QRect imgRect = imageRect();
    if (!image_.isNull()) {
        painter.drawImage(imgRect, image_);
    } else {
        painter.setPen(QColor(theme_.muted_text.isEmpty() ? "#9aa7bd" : theme_.muted_text));
        painter.drawText(rect(), Qt::AlignCenter | Qt::TextWordWrap,
                          "Upload an image, then click-drag here to set the flow direction.");
    }

    // The snapped arrow itself -- drawn across the image's own rect, not
    // the raw drag, so what's shown always matches what will actually run.
    QPointF center = imgRect.center();
    double half = std::min(imgRect.width(), imgRect.height()) * 0.35;
    QPointF dir;
    switch (direction_) {
        case ImageFlowDirection::Right: dir = {1, 0}; break;
        case ImageFlowDirection::Left: dir = {-1, 0}; break;
        case ImageFlowDirection::Up: dir = {0, -1}; break;
        case ImageFlowDirection::Down: dir = {0, 1}; break;
    }
    QPointF tail = center - dir * half;
    QPointF tip = center + dir * half;

    QColor arrowColor(theme_.accent.isEmpty() ? "#4fd1ff" : theme_.accent);
    QPen pen(arrowColor, 4, Qt::SolidLine, Qt::RoundCap);
    painter.setPen(pen);
    painter.drawLine(tail, tip);

    // Arrowhead: two short lines back from the tip at +/-25 degrees.
    double angle = std::atan2(dir.y(), dir.x());
    double headLen = 16.0;
    for (double da : {2.6, -2.6}) { // ~150 degrees from the shaft, i.e. a 30-degree-wide head
        QPointF headPt = tip - QPointF(std::cos(angle + da), std::sin(angle + da)) * headLen;
        painter.drawLine(tip, headPt);
    }
}

void DirectionArrowWidget::mousePressEvent(QMouseEvent* event) {
    dragging_ = true;
    dragStart_ = event->pos();
}

void DirectionArrowWidget::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) return;
    QPoint delta = event->pos() - dragStart_;
    if (delta.manhattanLength() < kMinDragPixels) return;
    auto newDir = snapToCardinal(delta);
    if (newDir != direction_) {
        direction_ = newDir;
        update();
        emit directionChanged(direction_);
    }
}

void DirectionArrowWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (dragging_) mouseMoveEvent(event); // catch a fast drag-then-release with no intermediate move events
    dragging_ = false;
}
