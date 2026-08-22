#pragma once
#include <QDateTime>
#include <QHash>
#include <QMainWindow>
#include <QQmlContext>
#include <QQueue>
#include <QQuickWidget>
#include <QString>
#include <QTcpSocket>
#include <QTimer>
#include <QProcess>
#include <QWebEngineView>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <nlohmann/json.hpp>

#include "QRangeSlider.hpp"

using json = nlohmann::json;

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE



class SensorData : public QObject
{
	Q_OBJECT
	Q_PROPERTY(qreal pitch READ pitch NOTIFY pitchChanged)
	Q_PROPERTY(qreal yaw READ yaw NOTIFY yawChanged)
	Q_PROPERTY(qreal roll READ roll NOTIFY rollChanged)

public:
	qreal pitch() const { return m_pitch; }
	qreal yaw() const { return m_yaw; }
	qreal roll() const { return m_roll; }

	void update(qreal p, qreal y, qreal r)
	{
		m_pitch = p;
		m_yaw	= y;
		m_roll	= r;
		emit pitchChanged();
		emit yawChanged();
		emit rollChanged();
	}

signals:
	void pitchChanged();
	void yawChanged();
	void rollChanged();

private:
	qreal m_pitch = 0, m_yaw = 0, m_roll = 0;
};


class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = nullptr);
	~MainWindow();

private:
	Ui::MainWindow* ui;

	QTcpSocket* socket;
	bool isEspVerified = false;

	int xCounter = 0;
	QQueue<QString> statusQueue;
	QTimer* statusTimer;
	QHash<QString, float> firmwareParams;

	void showNextStatusMessage();
	void enqueueStatusMessage(const QString& msg);
	void readFirmwareVars();
	void sendToEsp(const QString& data);
	void onReadyRead();
	void requestConfig();

	QTimer* updatingSeriesTimer;
	void updateSeriesByTimer();
	void updateTempHumSeries();
	void updateSeries(QChartView* view, int seriesIndex, float value);

	QChartView* createStyledChart(QWidget* widget,
		const QString& titleX,
		const QString& titleY,
		float minY,
		float maxY,
		const QStringList& seriesNames);
	QChartView* pwmChartView;
	QChartView* deltaPWMChartView;
	QChartView* accelerationChartView;
	QChartView* angularVelocityChartView;
	QChartView* temperatureChartView;
	QChartView* humidityChartView;

	QQuickWidget* quickWidget;
	SensorData sensorData;

	void updateGizmo();

	void updateChannels();
	QMap<QString, QRangeSlider*> sliders_;
	QRangeSlider* createSlider(
		QWidget* container, const int min, const int max);
	void setupRangeSliders();

QWebEngineView* mapView = nullptr;
QProcess *tileServer = nullptr;
QProcess *mapServer = nullptr;
void updateMap();


void WifiSsid();
QString getWifiInterfaceName();
};
