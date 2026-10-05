#pragma once

#include <QImage>
#include <QWidget>

#include "solvers/image_scenario_2d.hpp"
#include "theme.hpp"

// Shows the uploaded custom-scenario image (or a placeholder prompt before
// one is chosen) and lets the user pick the inflow direction by
// click-dragging an arrow across it. The solver can only ever run
// left-to-right internally (see image_scenario_2d.hpp), so the raw drag
// vector is snapped to the nearest of the 4 cardinal directions rather
// than kept as a free angle -- the arrow drawn back on screen always
// reflects the actual snapped direction, never the raw drag, so what the
// user sees is exactly what will run.
class DirectionArrowWidget : public QWidget {
    Q_OBJECT

public:
    explicit DirectionArrowWidget(QWidget* parent = nullptr);

    void setImage(const QImage& image);
    void setTheme(const Theme& theme);
    [[nodiscard]] cfd::solvers::ImageFlowDirection direction() const { return direction_; }

signals:
    void directionChanged(cfd::solvers::ImageFlowDirection dir);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    [[nodiscard]] QRect imageRect() const; // the letterboxed image rect within the widget

    QImage image_;
    Theme theme_;
    cfd::solvers::ImageFlowDirection direction_ = cfd::solvers::ImageFlowDirection::Right;
    bool dragging_ = false;
    QPoint dragStart_;
};
