#pragma once

#include <memory>

#include <QDialog>

#include "io/results_cache_reader.hpp"
#include "mesh/mesh.hpp"
#include "theme.hpp"

class QComboBox;
class QRadioButton;
class QCheckBox;
class QSlider;
class QPushButton;
class QLabel;
class QTimer;
class ResultsViewerWidget;
class ColorLegendWidget;

// Non-modal ("show()", not "exec()" -- the user can keep using the rest of
// the app while this is open) window hosting a ResultsViewerWidget plus
// scalar-field/slice-axis/slice-position controls and a timeline
// scrubber. Phase 1 ("walking skeleton") of the in-app results
// visualizer. The caller (ThreeDPanel) is expected to have already
// constructed `reader` itself -- opening a results cache directory can
// throw (missing/corrupt manifest), and that's easier to report as a
// QMessageBox before this dialog ever exists than from inside it.
class ResultsViewerDialog : public QDialog {
    Q_OBJECT

public:
    ResultsViewerDialog(const cfd::mesh::Mesh& mesh, std::shared_ptr<cfd::io::ResultsCacheReader> reader,
                         const Theme& theme, QWidget* parent = nullptr);

private slots:
    void onScalarFieldChanged(int index);
    void onAxisToggled();
    void onSlicePositionChanged(int value);
    void onFrameSliderChanged(int value);
    void onPlayPauseClicked();
    void onTimerTick();

private:
    void buildUi(const cfd::mesh::Mesh& mesh, const Theme& theme);

    std::shared_ptr<cfd::io::ResultsCacheReader> reader_;

    ResultsViewerWidget* viewer_ = nullptr;
    ColorLegendWidget* legend_ = nullptr;
    QComboBox* fieldCombo_ = nullptr;
    QRadioButton* axisXRadio_ = nullptr;
    QRadioButton* axisYRadio_ = nullptr;
    QRadioButton* axisZRadio_ = nullptr;
    QSlider* sliceSlider_ = nullptr;
    QCheckBox* streamlinesCheckbox_ = nullptr;
    QCheckBox* arrowsCheckbox_ = nullptr;
    QSlider* densitySlider_ = nullptr;
    QCheckBox* vortexVolumeCheckbox_ = nullptr;
    QSlider* frameSlider_ = nullptr;
    QLabel* frameTimeLabel_ = nullptr;
    QPushButton* playPauseButton_ = nullptr;
    QTimer* playTimer_ = nullptr;
};
