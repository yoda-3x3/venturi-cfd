#include "tab_2d.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QFrame>
#include <QMap>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>

#include <string>

#include "image_scenario_loader.hpp"
#include "paraview_launcher.hpp"
#include "solvers/image_scenario_2d.hpp"
#include "solvers/scenario_presets_2d.hpp"
#include "widgets/direction_arrow_widget.hpp"
#include "widgets/plot_widget.hpp"
#include "widgets/wrapped_label.hpp"

namespace {
constexpr const char* kCustomImageKey = "custom_image";
}

namespace {
struct ScenarioUiInfo {
    QString label, description;
};

const QMap<QString, ScenarioUiInfo>& scenarioUiInfo() {
    static const QMap<QString, ScenarioUiInfo> info = {
        {"cavity",
         {"Lid-Driven Cavity", "Classic benchmark: a square cavity with all walls stationary except the top, "
                                "which slides at a constant speed, driving a recirculating vortex."}},
        {"channel", {"Channel Flow", "Uniform flow enters a horizontal channel with no-slip top and bottom walls "
                                      "and develops toward a parabolic (Poiseuille) profile downstream."}},
        {"obstacle",
         {"Flow Past a Wall-Mounted Obstacle", "Channel flow over a rectangular bump mounted on the bottom wall, "
                                                "producing flow acceleration over the top and a recirculation zone "
                                                "downstream."}},
    };
    return info;
}
} // namespace

TwoDPanel::TwoDPanel(QWidget* parent) : QWidget(parent), settings_("VenturiCFD", "VenturiCFD") {
    buildUi();
}

void TwoDPanel::setTheme(const Theme& theme) {
    fieldPlot_->setTheme(theme);
    residualPlot_->setTheme(theme);
    directionWidget_->setTheme(theme);
}

void TwoDPanel::shutdown() {
    if (worker_) worker_->requestStop();
    if (thread_ && thread_->isRunning()) {
        thread_->quit();
        thread_->wait(3000);
    }
}

void TwoDPanel::buildUi() {
    auto* root = new QHBoxLayout(this);

    // ---- Left panel ----
    // Scrolled rather than a plain fixed-width widget so a window shorter
    // than the panel's natural content height scrolls instead of
    // squeezing every control down to an illegible size (same fix as
    // ThreeDPanel's left panel).
    auto* leftScroll = new QScrollArea(this);
    leftScroll->setWidgetResizable(true);
    leftScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    leftScroll->setFrameShape(QFrame::NoFrame);
    // A *minimum*, not a fixed width: at larger UiScale settings, labels
    // like "Reynolds number:" need more horizontal room than a hardcoded
    // 320px allows -- a fixed width just clipped the text. Left unbounded
    // above the minimum, the scroll area takes leftPanel's own sizeHint
    // width (QHBoxLayout gives it exactly that, since only rightPanel below
    // has a stretch factor), which grows with the font automatically.
    leftScroll->setMinimumWidth(320);

    auto* leftPanel = new QWidget(leftScroll);
    leftPanel->setMinimumWidth(300);
    auto* leftLayout = new QVBoxLayout(leftPanel);

    auto* caseGroup = new QGroupBox("Case Setup", leftPanel);
    auto* caseForm = new QFormLayout(caseGroup);

    scenarioCombo_ = new QComboBox(caseGroup);
    for (const auto& key : cfd::solvers::scenario_keys_2d()) {
        QString qkey = QString::fromStdString(key);
        scenarioCombo_->addItem(scenarioUiInfo().value(qkey).label, qkey);
    }
    // Not one of scenario_keys_2d()'s preset-driven entries -- it has no
    // Re/U/obstacle-geometry defaults to look up, just an uploaded image
    // and a drawn direction, both handled entirely in this tab (see
    // onRunClicked/onUploadImage).
    scenarioCombo_->addItem("Custom Image", QString(kCustomImageKey));
    caseForm->addRow("Scenario:", scenarioCombo_);

    descriptionLabel_ = new WrappedLabel(caseGroup);
    descriptionLabel_->setObjectName("description");
    caseForm->addRow(descriptionLabel_);

    auto* gridRow = new QWidget(caseGroup);
    auto* gridLayout = new QHBoxLayout(gridRow);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    nxSpin_ = new QSpinBox(gridRow);
    nxSpin_->setRange(11, 400);
    nySpin_ = new QSpinBox(gridRow);
    nySpin_->setRange(11, 400);
    gridLayout->addWidget(nxSpin_);
    gridLayout->addWidget(new QLabel("x", gridRow));
    gridLayout->addWidget(nySpin_);
    caseForm->addRow("Grid points:", gridRow);

    reSpin_ = new QDoubleSpinBox(caseGroup);
    reSpin_->setRange(1, 100000);
    reSpin_->setDecimals(0);
    reSpin_->setSingleStep(50);
    caseForm->addRow("Reynolds number:", reSpin_);

    uSpin_ = new QDoubleSpinBox(caseGroup);
    uSpin_->setRange(0.01, 50);
    uSpin_->setDecimals(2);
    caseForm->addRow("Lid/inflow speed:", uSpin_);

    leftLayout->addWidget(caseGroup);

    obstacleGroup_ = new QGroupBox("Obstacle Geometry", leftPanel);
    auto* obstacleForm = new QFormLayout(obstacleGroup_);
    obstacleX0Spin_ = new QDoubleSpinBox(obstacleGroup_);
    obstacleX0Spin_->setRange(0, 100);
    obstacleX0Spin_->setDecimals(3);
    obstacleWidthSpin_ = new QDoubleSpinBox(obstacleGroup_);
    obstacleWidthSpin_->setRange(0.001, 100);
    obstacleWidthSpin_->setDecimals(3);
    obstacleHeightSpin_ = new QDoubleSpinBox(obstacleGroup_);
    obstacleHeightSpin_->setRange(0.001, 100);
    obstacleHeightSpin_->setDecimals(3);
    obstacleForm->addRow("x0:", obstacleX0Spin_);
    obstacleForm->addRow("Width:", obstacleWidthSpin_);
    obstacleForm->addRow("Height:", obstacleHeightSpin_);
    leftLayout->addWidget(obstacleGroup_);

    imageGroup_ = new QGroupBox("Custom Image", leftPanel);
    auto* imageLayout = new QVBoxLayout(imageGroup_);
    uploadImageButton_ = new QPushButton("Upload Image...", imageGroup_);
    imageLayout->addWidget(uploadImageButton_);
    imageFileLabel_ = new WrappedLabel(imageGroup_);
    imageFileLabel_->setObjectName("description");
    imageFileLabel_->setText("No image chosen.");
    imageLayout->addWidget(imageFileLabel_);
    directionWidget_ = new DirectionArrowWidget(imageGroup_);
    imageLayout->addWidget(directionWidget_);
    auto* thresholdRow = new QWidget(imageGroup_);
    auto* thresholdLayout = new QHBoxLayout(thresholdRow);
    thresholdLayout->setContentsMargins(0, 0, 0, 0);
    thresholdLayout->addWidget(new QLabel("Solid threshold:", thresholdRow));
    imageThresholdSpin_ = new QDoubleSpinBox(thresholdRow);
    imageThresholdSpin_->setRange(0.02, 0.98);
    imageThresholdSpin_->setSingleStep(0.05);
    imageThresholdSpin_->setDecimals(2);
    imageThresholdSpin_->setValue(0.5);
    thresholdLayout->addWidget(imageThresholdSpin_);
    imageLayout->addWidget(thresholdRow);
    leftLayout->addWidget(imageGroup_);
    connect(uploadImageButton_, &QPushButton::clicked, this, &TwoDPanel::onUploadImage);
    connect(imageThresholdSpin_, &QDoubleSpinBox::valueChanged, this, &TwoDPanel::updateImagePreviewMask);
    connect(nxSpin_, &QSpinBox::valueChanged, this, &TwoDPanel::updateImagePreviewMask);
    connect(nySpin_, &QSpinBox::valueChanged, this, &TwoDPanel::updateImagePreviewMask);

    auto* runGroup = new QGroupBox("Run", leftPanel);
    auto* runForm = new QFormLayout(runGroup);
    stepsSpin_ = new QSpinBox(runGroup);
    stepsSpin_->setRange(50, 500000);
    stepsSpin_->setSingleStep(500);
    stepsSpin_->setValue(3000);
    outputEverySpin_ = new QSpinBox(runGroup);
    outputEverySpin_->setRange(1, 5000);
    outputEverySpin_->setValue(20);
    runForm->addRow("Steps:", stepsSpin_);
    runForm->addRow("Output every:", outputEverySpin_);

    auto* outDirRow = new QWidget(runGroup);
    auto* outDirLayout = new QHBoxLayout(outDirRow);
    outDirLayout->setContentsMargins(0, 0, 0, 0);
    outputDirEdit_ = new QLineEdit(outDirRow);
    auto* browseButton = new QPushButton("Browse...", outDirRow);
    outDirLayout->addWidget(outputDirEdit_);
    outDirLayout->addWidget(browseButton);
    runForm->addRow("Output folder:", outDirRow);
    connect(browseButton, &QPushButton::clicked, this, &TwoDPanel::onBrowseOutputDir);

    auto* buttonRow = new QWidget(runGroup);
    auto* buttonLayout = new QHBoxLayout(buttonRow);
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    runButton_ = new QPushButton("Run Simulation", buttonRow);
    stopButton_ = new QPushButton("Stop", buttonRow);
    stopButton_->setObjectName("stopButton");
    stopButton_->setEnabled(false);
    buttonLayout->addWidget(runButton_);
    buttonLayout->addWidget(stopButton_);
    runForm->addRow(buttonRow);

    progressBar_ = new QProgressBar(runGroup);
    progressBar_->setRange(0, 100);
    runForm->addRow(progressBar_);

    statusLabel_ = new QLabel("Idle.", runGroup);
    runForm->addRow(statusLabel_);

    paraviewButton_ = new QPushButton("Open Latest Result in ParaView", runGroup);
    paraviewButton_->setObjectName("paraviewButton");
    paraviewButton_->setEnabled(false);
    runForm->addRow(paraviewButton_);

    leftLayout->addWidget(runGroup);
    leftLayout->addStretch();

    connect(scenarioCombo_, &QComboBox::currentIndexChanged, this, &TwoDPanel::onScenarioChanged);
    connect(runButton_, &QPushButton::clicked, this, &TwoDPanel::onRunClicked);
    connect(stopButton_, &QPushButton::clicked, this, &TwoDPanel::onStopClicked);
    connect(paraviewButton_, &QPushButton::clicked, this, &TwoDPanel::onOpenInParaView);

    // ---- Right panel ----
    auto* rightPanel = new QWidget(this);
    auto* rightLayout = new QVBoxLayout(rightPanel);
    fieldPlot_ = new PlotWidget(PlotWidget::Mode::Heatmap, rightPanel);
    residualPlot_ = new PlotWidget(PlotWidget::Mode::LogLine, rightPanel);
    rightLayout->addWidget(fieldPlot_, 2);
    rightLayout->addWidget(residualPlot_, 1);

    leftScroll->setWidget(leftPanel);
    root->addWidget(leftScroll);
    root->addWidget(rightPanel, 1);

    onScenarioChanged();
}

void TwoDPanel::onScenarioChanged() {
    QString key = scenarioCombo_->currentData().toString();
    bool isCustomImage = key == QString(kCustomImageKey);
    obstacleGroup_->setVisible(false);
    imageGroup_->setVisible(isCustomImage);

    if (isCustomImage) {
        // Not preset-driven (see the combo-box setup in buildUi) -- no
        // scenario_preset_2d() lookup here, it only knows the 3 built-in
        // keys and would throw for this one.
        descriptionLabel_->setText(
            "Upload a PNG/JPG/WEBP image: dark, opaque pixels become a solid obstacle, everything else is "
            "fluid. Click-drag on the preview below to set which way the flow enters.");
        nxSpin_->setValue(121);
        nySpin_->setValue(81);
        reSpin_->setValue(150.0);
        uSpin_->setValue(1.0);
    } else {
        std::string keyStd = key.toStdString();
        const auto& preset = cfd::solvers::scenario_preset_2d(keyStd);
        descriptionLabel_->setText(scenarioUiInfo().value(key).description);
        nxSpin_->setValue(preset.default_nx);
        nySpin_->setValue(preset.default_ny);
        reSpin_->setValue(preset.default_Re);
        uSpin_->setValue(preset.default_U);
        obstacleGroup_->setVisible(preset.has_obstacle);
        if (preset.has_obstacle) {
            auto obstacle = cfd::solvers::default_obstacle_2d(preset);
            obstacleX0Spin_->setValue(obstacle.x0);
            obstacleWidthSpin_->setValue(obstacle.width);
            obstacleHeightSpin_->setValue(obstacle.height);
        }
    }
    suggestOutputDir();
}

void TwoDPanel::onUploadImage() {
    QString path = QFileDialog::getOpenFileName(this, "Choose an Image", QString(),
                                                  "Images (*.png *.jpg *.jpeg *.webp)");
    if (path.isEmpty()) return;

    auto image = image_scenario::loadImage(path);
    if (!image) {
        QMessageBox::warning(this, "Couldn't Load Image",
                              QString("Couldn't read %1 as an image.").arg(QFileInfo(path).fileName()));
        return;
    }
    customImage_ = *image;
    imageFileLabel_->setText(QFileInfo(path).fileName());
    updateImagePreviewMask();
}

void TwoDPanel::updateImagePreviewMask() {
    if (customImage_.isNull()) return;
    int nx = nxSpin_->value(), ny = nySpin_->value();
    auto mask = image_scenario::rasterizeToMask(customImage_, nx, ny, imageThresholdSpin_->value());

    // Shown in the arrow widget instead of the raw upload so the user sees
    // exactly what will become solid vs. fluid at the current threshold
    // and grid resolution, before spending a run on it.
    QImage preview(nx, ny, QImage::Format_RGB32);
    for (int j = 0; j < ny; ++j) {
        int row = ny - 1 - j; // undo rasterizeToMask's bottom-origin flip for on-screen display
        for (int i = 0; i < nx; ++i) {
            bool solid = mask[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)] != 0;
            preview.setPixelColor(i, row, solid ? QColor(0x1a, 0x22, 0x33) : QColor(0xe8, 0xec, 0xf4));
        }
    }
    directionWidget_->setImage(preview);
}

void TwoDPanel::suggestOutputDir() {
    QString key = scenarioCombo_->currentData().toString();
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString dir = QDir::current().filePath(QString("runs/%1_%2").arg(key, timestamp));
    outputDirEdit_->setText(QDir::toNativeSeparators(dir));
}

void TwoDPanel::onBrowseOutputDir() {
    QString dir = QFileDialog::getExistingDirectory(this, "Select Output Folder", outputDirEdit_->text());
    if (!dir.isEmpty()) outputDirEdit_->setText(dir);
}

void TwoDPanel::setControlsEnabled(bool running) {
    scenarioCombo_->setEnabled(!running);
    nxSpin_->setEnabled(!running);
    nySpin_->setEnabled(!running);
    reSpin_->setEnabled(!running);
    uSpin_->setEnabled(!running);
    obstacleX0Spin_->setEnabled(!running);
    obstacleWidthSpin_->setEnabled(!running);
    obstacleHeightSpin_->setEnabled(!running);
    uploadImageButton_->setEnabled(!running);
    imageThresholdSpin_->setEnabled(!running);
    directionWidget_->setEnabled(!running);
    stepsSpin_->setEnabled(!running);
    outputEverySpin_->setEnabled(!running);
    outputDirEdit_->setEnabled(!running);
    runButton_->setEnabled(!running);
    stopButton_->setEnabled(running);
}

void TwoDPanel::onRunClicked() {
    if (thread_ && thread_->isRunning()) return;
    if (outputDirEdit_->text().isEmpty()) {
        statusLabel_->setText("Please choose an output folder.");
        return;
    }

    QString key = scenarioCombo_->currentData().toString();
    std::string keyStd = key.toStdString();
    bool isCustomImage = key == QString(kCustomImageKey);

    cfd::pipeline::Run2DOptions opts;
    opts.scenario = keyStd;
    opts.Re = reSpin_->value();
    opts.U = uSpin_->value();

    if (isCustomImage) {
        if (customImage_.isNull()) {
            statusLabel_->setText("Please upload an image first.");
            return;
        }
        int nx = nxSpin_->value(), ny = nySpin_->value();
        auto displayMask = image_scenario::rasterizeToMask(customImage_, nx, ny, imageThresholdSpin_->value());
        auto oriented = cfd::solvers::orient_mask_for_solver(displayMask, nx, ny, directionWidget_->direction());
        opts.custom_mask = oriented.mask;
        opts.custom_mask_nx = oriented.nx;
        opts.custom_mask_ny = oriented.ny;
        opts.image_direction = directionWidget_->direction();
    } else {
        const auto& preset = cfd::solvers::scenario_preset_2d(keyStd);
        opts.nx = nxSpin_->value();
        opts.ny = nySpin_->value();
        if (preset.has_obstacle) {
            opts.obstacle_x0 = obstacleX0Spin_->value();
            opts.obstacle_width = obstacleWidthSpin_->value();
            opts.obstacle_height = obstacleHeightSpin_->value();
        }
    }
    opts.n_steps = stepsSpin_->value();
    opts.output_every = outputEverySpin_->value();
    opts.output_dir = outputDirEdit_->text().toStdString();
    opts.case_name = key.toStdString();

    fieldPlot_->setHeatmapData({}, 0, 0);
    residualPlot_->clearLine();
    paraviewButton_->setEnabled(false);

    worker_ = new Sim2DWorker(opts);
    thread_ = new QThread(this);
    worker_->moveToThread(thread_);

    connect(thread_, &QThread::started, worker_, &Sim2DWorker::run);
    connect(worker_, &Sim2DWorker::progressChanged, this, &TwoDPanel::onProgress);
    connect(worker_, &Sim2DWorker::previewReady, this, &TwoDPanel::onPreview);
    connect(worker_, &Sim2DWorker::finished, this, &TwoDPanel::onFinished);
    connect(worker_, &Sim2DWorker::stopped, this, &TwoDPanel::onStopped);
    connect(worker_, &Sim2DWorker::errorOccurred, this, &TwoDPanel::onErrorOccurred);
    connect(worker_, &Sim2DWorker::finished, thread_, &QThread::quit);
    connect(worker_, &Sim2DWorker::stopped, thread_, &QThread::quit);
    connect(worker_, &Sim2DWorker::errorOccurred, thread_, &QThread::quit);
    connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);

    setControlsEnabled(true);
    statusLabel_->setText("Running...");
    progressBar_->setValue(0);

    thread_->start();
}

void TwoDPanel::onStopClicked() {
    if (worker_) worker_->requestStop();
}

void TwoDPanel::onProgress(int step, double residual) {
    int total = stepsSpin_->value();
    int pct = total > 0 ? static_cast<int>(100.0 * step / total) : 0;
    progressBar_->setValue(pct);
    statusLabel_->setText(QString("Step %1 / %2, residual %3").arg(step).arg(total).arg(residual, 0, 'e', 3));
    residualPlot_->appendResidualPoint(step, residual);
}

void TwoDPanel::onPreview(Preview2DSnapshot snapshot) {
    if (!snapshot.fields) return;
    fieldPlot_->setHeatmapData(snapshot.fields->velocity_magnitude, snapshot.nx, snapshot.ny, "velocity_magnitude");
    fieldPlot_->setObstacleMask(snapshot.fields->obstacle);
}

void TwoDPanel::onFinished(QString pvdPath) {
    lastPvdPath_ = pvdPath;
    statusLabel_->setText(QString("Simulation complete: %1").arg(pvdPath));
    setControlsEnabled(false);
    paraviewButton_->setEnabled(true);
    progressBar_->setValue(100);
}

void TwoDPanel::onStopped() {
    statusLabel_->setText("Stopped.");
    setControlsEnabled(false);
}

void TwoDPanel::onErrorOccurred(QString message) {
    statusLabel_->setText(QString("Error: %1").arg(message));
    setControlsEnabled(false);
}

void TwoDPanel::onOpenInParaView() {
    if (lastPvdPath_.isEmpty()) return;
    QString exe = settings_.value("paraviewPath").toString();
    if (exe.isEmpty() || !QFileInfo::exists(exe)) {
        auto candidates = paraview_launcher::findCandidates();
        if (candidates.isEmpty()) {
            QMessageBox::warning(this, "ParaView Not Found",
                                  "Could not locate ParaView automatically. Please install it and try again.");
            return;
        }
        exe = candidates.first();
        settings_.setValue("paraviewPath", exe);
    }
    if (!paraview_launcher::launch(exe, lastPvdPath_)) {
        QMessageBox::warning(this, "Launch Failed", "Failed to launch ParaView.");
    }
}
