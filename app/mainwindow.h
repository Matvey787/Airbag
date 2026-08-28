#pragma once

#include <QMainWindow>
#include <QTimer>

#include "chartshandler.h"
#include "firmwarelink.h"
#include "gizmocontroller.h"
#include "mapcontroller.h"
#include "rangesliderspanel.h"
#include "firmwarevarshandler.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow();

private:
	void updateOnTimer();
	void updateTempHumLabels();

	Ui::MainWindow* ui;

FirmwareVarsHandler* firmwareHandler;
	FirmwareLink* firmwareLink_;
ChartsHandler* chartsHandler_;
	RangeSlidersPanel* rangeSliders_;
	MapController* mapController_;
	GizmoController* gizmoController_;

	QTimer* updatingSeriesTimer;
};
