#pragma once

#include <QHash>
#include <QMap>
#include <QObject>
#include <QString>

class QEvent;
class QWidget;
class QRangeSlider;

namespace Ui
{
class MainWindow;
}

// Owns the seven QRangeSlider channel widgets, their linked min/max spin
// boxes, and the "locked" overlay shown while channel editing is disabled.
class RangeSlidersPanel : public QObject
{
	Q_OBJECT

public:
	explicit RangeSlidersPanel(Ui::MainWindow* ui, QObject* parent = nullptr);
void onParamUpdated(const QString& key, float value);

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;

private:
	QRangeSlider* createSlider(QWidget* container, int min, int max);
struct sliderData {
QString markerFirmVar;
QString minFirmVar;
QString maxFirmVar;

QRangeSlider* slider;
};

using sliderName = QString;

	Ui::MainWindow* ui_;
QMap<sliderName, sliderData> sliders_;
	QWidget* channelsOverlay_ = nullptr;
};
