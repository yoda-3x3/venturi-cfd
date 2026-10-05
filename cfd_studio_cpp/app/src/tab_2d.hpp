#pragma once

#include <QImage>
#include <QSettings>
#include <QThread>
#include <QWidget>

#include "sim2d_worker.hpp"
#include "theme.hpp"

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QLineEdit;
class QPushButton;
class QProgressBar;
class QLabel;
class QGroupBox;
class PlotWidget;
class DirectionArrowWidget;

// Port of ui/main_window.py's inline 2D-tab construction (_build_left_panel
// / _build_right_panel / worker lifecycle) -- factored into its own widget
// class here rather than left inline in MainWindow, for the same reason
// the 3D tab gets its own ThreeDPanel class in Python: it's a big,
// self-contained chunk of UI + worker-thread plumbing.
class TwoDPanel : public QWidget {
    Q_OBJECT

public:
    explicit TwoDPanel(QWidget* parent = nullptr);

    void setTheme(const Theme& theme);

    // Requests the running simulation (if any) stop and waits briefly --
    // called from MainWindow::closeEvent, mirroring
    // ui/main_window.py's closeEvent + ThreeDPanel.shutdown().
    void shutdown();

private slots:
    void onScenarioChanged();
    void onBrowseOutputDir();
    void onRunClicked();
    void onStopClicked();
    void onProgress(int step, double residual);
    void onPreview(Preview2DSnapshot snapshot);
    void onFinished(QString pvdPath);
    void onStopped();
    void onErrorOccurred(QString message);
    void onOpenInParaView();
    void onUploadImage();

private:
    void buildUi();
    void setControlsEnabled(bool running);
    void suggestOutputDir();
    void updateImagePreviewMask();

    QComboBox* scenarioCombo_ = nullptr;
    QLabel* descriptionLabel_ = nullptr;
    QSpinBox* nxSpin_ = nullptr;
    QSpinBox* nySpin_ = nullptr;
    QDoubleSpinBox* reSpin_ = nullptr;
    QDoubleSpinBox* uSpin_ = nullptr;
    QGroupBox* obstacleGroup_ = nullptr;
    QDoubleSpinBox* obstacleX0Spin_ = nullptr;
    QDoubleSpinBox* obstacleWidthSpin_ = nullptr;
    QDoubleSpinBox* obstacleHeightSpin_ = nullptr;
    QGroupBox* imageGroup_ = nullptr;
    QPushButton* uploadImageButton_ = nullptr;
    QLabel* imageFileLabel_ = nullptr;
    QDoubleSpinBox* imageThresholdSpin_ = nullptr;
    DirectionArrowWidget* directionWidget_ = nullptr;
    QImage customImage_;
    QSpinBox* stepsSpin_ = nullptr;
    QSpinBox* outputEverySpin_ = nullptr;
    QLineEdit* outputDirEdit_ = nullptr;
    QPushButton* runButton_ = nullptr;
    QPushButton* stopButton_ = nullptr;
    QProgressBar* progressBar_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QPushButton* paraviewButton_ = nullptr;

    PlotWidget* fieldPlot_ = nullptr;
    PlotWidget* residualPlot_ = nullptr;

    QThread* thread_ = nullptr;
    Sim2DWorker* worker_ = nullptr;
    QString lastPvdPath_;
    QSettings settings_;
};
