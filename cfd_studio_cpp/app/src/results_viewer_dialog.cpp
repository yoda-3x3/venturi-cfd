#include "results_viewer_dialog.hpp"

#include <algorithm>

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include "widgets/color_legend_widget.hpp"
#include "widgets/results_viewer_widget.hpp"

namespace {
constexpr int kSlicePositionSteps = 1000; // slider resolution, mapped to [0,1]
constexpr int kPlaybackIntervalMs = 400;
} // namespace

ResultsViewerDialog::ResultsViewerDialog(const cfd::mesh::Mesh& mesh, std::shared_ptr<cfd::io::ResultsCacheReader> reader,
                                          const Theme& theme, QWidget* parent)
    : QDialog(parent), reader_(std::move(reader)) {
    setAttribute(Qt::WA_DeleteOnClose);
    buildUi(mesh, theme);
}

void ResultsViewerDialog::buildUi(const cfd::mesh::Mesh& mesh, const Theme& theme) {
    setWindowTitle("Results Viewer");
    resize(1030, 680);

    auto* root = new QHBoxLayout(this);

    viewer_ = new ResultsViewerWidget(this);
    viewer_->setTheme(theme);
    viewer_->setMesh(mesh);
    viewer_->setResultsCache(reader_);
    root->addWidget(viewer_, 1);

    legend_ = new ColorLegendWidget(this);
    legend_->setTheme(theme);
    legend_->setFieldName("Velocity magnitude"); // matches fieldCombo_'s default (index 0) below
    root->addWidget(legend_);
    connect(viewer_, &ResultsViewerWidget::valueRangeChanged, legend_, &ColorLegendWidget::setRange);

    auto* side = new QWidget(this);
    side->setFixedWidth(260);
    auto* sideLayout = new QVBoxLayout(side);

    auto* fieldForm = new QFormLayout();
    fieldCombo_ = new QComboBox(side);
    fieldCombo_->addItem("Velocity magnitude");
    fieldCombo_->addItem("Pressure");
    fieldCombo_->addItem("Velocity U");
    fieldCombo_->addItem("Velocity V");
    fieldCombo_->addItem("Velocity W");
    fieldForm->addRow("Field:", fieldCombo_);
    sideLayout->addLayout(fieldForm);
    connect(fieldCombo_, &QComboBox::currentIndexChanged, this, &ResultsViewerDialog::onScalarFieldChanged);

    sideLayout->addWidget(new QLabel("Slice axis:", side));
    auto* axisRow = new QWidget(side);
    auto* axisLayout = new QHBoxLayout(axisRow);
    axisLayout->setContentsMargins(0, 0, 0, 0);
    axisXRadio_ = new QRadioButton("X", axisRow);
    axisYRadio_ = new QRadioButton("Y", axisRow);
    axisZRadio_ = new QRadioButton("Z", axisRow);
    axisYRadio_->setChecked(true); // matches ResultsViewerWidget's own default (Axis::Y)
    axisLayout->addWidget(axisXRadio_);
    axisLayout->addWidget(axisYRadio_);
    axisLayout->addWidget(axisZRadio_);
    sideLayout->addWidget(axisRow);
    connect(axisXRadio_, &QRadioButton::toggled, this, &ResultsViewerDialog::onAxisToggled);
    connect(axisYRadio_, &QRadioButton::toggled, this, &ResultsViewerDialog::onAxisToggled);
    connect(axisZRadio_, &QRadioButton::toggled, this, &ResultsViewerDialog::onAxisToggled);

    sideLayout->addWidget(new QLabel("Slice position:", side));
    sliceSlider_ = new QSlider(Qt::Horizontal, side);
    sliceSlider_->setRange(0, kSlicePositionSteps);
    sliceSlider_->setValue(kSlicePositionSteps / 2);
    sideLayout->addWidget(sliceSlider_);
    connect(sliceSlider_, &QSlider::valueChanged, this, &ResultsViewerDialog::onSlicePositionChanged);

    streamlinesCheckbox_ = new QCheckBox("Show streamlines", side);
    sideLayout->addWidget(streamlinesCheckbox_);
    connect(streamlinesCheckbox_, &QCheckBox::toggled, viewer_, &ResultsViewerWidget::setShowStreamlines);

    arrowsCheckbox_ = new QCheckBox("Show velocity arrows", side);
    sideLayout->addWidget(arrowsCheckbox_);
    connect(arrowsCheckbox_, &QCheckBox::toggled, viewer_, &ResultsViewerWidget::setShowVelocityArrows);

    sideLayout->addWidget(new QLabel("Density:", side));
    densitySlider_ = new QSlider(Qt::Horizontal, side);
    densitySlider_->setRange(2, 60);
    densitySlider_->setValue(18); // matches ResultsViewerWidget's own default density
    sideLayout->addWidget(densitySlider_);
    connect(densitySlider_, &QSlider::valueChanged, viewer_, &ResultsViewerWidget::setVectorDensity);

    vortexVolumeCheckbox_ = new QCheckBox("Show vortex cores (Q-criterion)", side);
    vortexVolumeCheckbox_->setToolTip(
        "Ray-marched volumetric render of rotation-dominated flow regions -- can be slow on lower-end GPUs.");
    sideLayout->addWidget(vortexVolumeCheckbox_);
    connect(vortexVolumeCheckbox_, &QCheckBox::toggled, viewer_, &ResultsViewerWidget::setShowVortexVolume);

    sideLayout->addStretch(1);

    sideLayout->addWidget(new QLabel("Timestep:", side));
    frameTimeLabel_ = new QLabel("t = 0", side);
    frameTimeLabel_->setObjectName("description");
    sideLayout->addWidget(frameTimeLabel_);

    int lastFrame = std::max(0, reader_->frame_count() - 1);
    frameSlider_ = new QSlider(Qt::Horizontal, side);
    frameSlider_->setRange(0, lastFrame);
    frameSlider_->setValue(0);
    frameSlider_->setEnabled(lastFrame > 0);
    sideLayout->addWidget(frameSlider_);
    connect(frameSlider_, &QSlider::valueChanged, this, &ResultsViewerDialog::onFrameSliderChanged);

    playPauseButton_ = new QPushButton("Play", side);
    playPauseButton_->setEnabled(lastFrame > 0);
    sideLayout->addWidget(playPauseButton_);
    connect(playPauseButton_, &QPushButton::clicked, this, &ResultsViewerDialog::onPlayPauseClicked);

    root->addWidget(side);

    playTimer_ = new QTimer(this);
    connect(playTimer_, &QTimer::timeout, this, &ResultsViewerDialog::onTimerTick);

    onFrameSliderChanged(0);
}

void ResultsViewerDialog::onScalarFieldChanged(int index) {
    viewer_->setScalarField(static_cast<ResultsViewerWidget::ScalarField>(index));
    legend_->setFieldName(fieldCombo_->currentText());
}

void ResultsViewerDialog::onAxisToggled() {
    ResultsViewerWidget::Axis axis = axisXRadio_->isChecked()   ? ResultsViewerWidget::Axis::X
                                      : axisZRadio_->isChecked() ? ResultsViewerWidget::Axis::Z
                                                                  : ResultsViewerWidget::Axis::Y;
    viewer_->setSliceAxis(axis);
}

void ResultsViewerDialog::onSlicePositionChanged(int value) {
    viewer_->setSlicePosition(static_cast<double>(value) / kSlicePositionSteps);
}

void ResultsViewerDialog::onFrameSliderChanged(int value) {
    viewer_->setFrame(value);
    if (reader_->frame_count() > 0) {
        frameTimeLabel_->setText(QString("t = %1").arg(reader_->frame_time(value), 0, 'g', 4));
    }
}

void ResultsViewerDialog::onPlayPauseClicked() {
    if (playTimer_->isActive()) {
        playTimer_->stop();
        playPauseButton_->setText("Play");
    } else {
        playTimer_->start(kPlaybackIntervalMs);
        playPauseButton_->setText("Pause");
    }
}

void ResultsViewerDialog::onTimerTick() {
    int next = frameSlider_->value() + 1;
    if (next > frameSlider_->maximum()) next = 0; // loop
    frameSlider_->setValue(next); // drives onFrameSliderChanged via its own valueChanged signal
}
