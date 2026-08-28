#pragma once

#include "firmwarevarshandler.h"

#include <QHash>
#include <QList>
#include <QPointF>
#include <QObject>
#include <QString>

class QLabel;
class QWidget;
class RouteWidget;

namespace Ui
{
class MainWindow;
}

class MapController : public QObject
{
	Q_OBJECT

public:
	explicit MapController(QWidget* mapContainer, QObject* parent = nullptr);
	~MapController() override;

	void refreshWifiSsid(QLabel* wifiField);
	void updateMap(const FirmwareVarsHandler& firmwareParamsHandler);
	void addLabelBinding(QLabel* label, const QString& key);

private:
	static QString getWifiInterfaceName();

	RouteWidget* mapView_;
	QPointF startPoint_;
	QList<QPointF> routePoints_;
	bool hasStart_ = false;

	struct LabelBinding
	{
		QLabel* label;
		QString key;
	};

	QList<LabelBinding> labelBindings_;
};
