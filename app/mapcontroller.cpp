#include "mapcontroller.h"

#include <QDebug>
#include <QLabel>
#include <QLineEdit>
#include <QPaintEvent>
#include <QPainter>
#include <QProcess>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>

#include "firmwarevarshandler.h"
#include "project_info.h"

class RouteWidget : public QWidget
{
	Q_OBJECT
public:
	explicit RouteWidget(QWidget* parent = nullptr) : QWidget(parent)
	{
		setStyleSheet("background: white; border: 1px solid #cfcfcf;");
		setAttribute(Qt::WA_OpaquePaintEvent);
	}

	void setRoute(const QPointF& start, const QList<QPointF>& points)
	{
		startPoint_	 = start;
		routePoints_ = points;
		update();
	}

protected:
	void paintEvent(QPaintEvent* event) override
	{
		Q_UNUSED(event);
		QPainter painter(this);
		painter.fillRect(rect(), Qt::white);

		QPen routePen(Qt::darkGray, 2);
		painter.setPen(routePen);
		if (routePoints_.size() > 1)
		{
			QVector<QPointF> pts(routePoints_.size());
			for (int i = 0; i < routePoints_.size(); ++i)
				pts[i] = routePoints_[i];
			painter.drawPolyline(pts.constData(), pts.size());
		}

		painter.setPen(Qt::NoPen);
		painter.setBrush(QColor(0, 170, 80));
		painter.drawEllipse(startPoint_, 7, 7);
		painter.setBrush(Qt::NoBrush);
		painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
		painter.drawEllipse(startPoint_, 18, 18);

		painter.setPen(QPen(Qt::lightGray, 1));
		for (int x = 0; x < width(); x += 20)
			painter.drawLine(x, 0, x, height());
		for (int y = 0; y < height(); y += 20)
			painter.drawLine(0, y, width(), y);
	}

private:
	QPointF startPoint_;
	QList<QPointF> routePoints_;
};

#include "mapcontroller.moc"

MapController::MapController(QWidget* mapContainer, QObject* parent) :
	QObject(parent)
{
	mapView_ = new RouteWidget(mapContainer);
	mapView_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

	auto* mapLayout = new QVBoxLayout(mapContainer);
	mapLayout->setContentsMargins(0, 0, 0, 0);
	mapLayout->addWidget(mapView_);
}

MapController::~MapController() = default;

QString MapController::getWifiInterfaceName()
{
	QProcess proc;
	proc.start("nmcli", { "-t", "-f", "DEVICE,TYPE", "dev", "status" });
	proc.waitForFinished(2000);
	QString output = QString::fromUtf8(proc.readAllStandardOutput());

	for (const QString& line : output.split('\n', Qt::SkipEmptyParts))
	{
		QStringList parts = line.split(':');
		if (parts.size() == 2 && parts[1] == "wifi")
			return parts[0];
	}
	return QString();
}

void MapController::refreshWifiSsid(QLabel* wifiField)
{
	QString iface = getWifiInterfaceName();
	if (iface.isEmpty())
	{
		wifiField->setText("Not connected");
		return;
	}

	QProcess proc;
	proc.start(
		"nmcli", { "-t", "-f", "GENERAL.CONNECTION", "dev", "show", iface });
	proc.waitForFinished(2000);
	QString output = QString::fromUtf8(proc.readAllStandardOutput()).trimmed();
	QString ssid   = output.section(':', 1);
	wifiField->setText(ssid.isEmpty() || ssid == "--" ? "Not connected" : ssid);
}


void MapController::addLabelBinding(QLabel* label, const QString& key)
{
	labelBindings_.append({ label, key });
}

void MapController::updateMap(const FirmwareVarsHandler& firmwareParamsHandler)
{
	for (const auto& binding : labelBindings_)
	{
		binding.label->setText(
			QString::number(firmwareParamsHandler[binding.key]));
	}

	const double lat = static_cast<double>(firmwareParamsHandler["GPS_LAT"]);
	const double lon = static_cast<double>(firmwareParamsHandler["GPS_LON"]);

	QPointF center = mapView_->rect().center();
	startPoint_	   = center;
	hasStart_	   = true;

	QPointF p;
	p.setX(startPoint_.x() + lon * 5.0);
	p.setY(startPoint_.y() - lat * 5.0);

	if (routePoints_.isEmpty())
	{
		routePoints_.append(startPoint_);
	}
	routePoints_.append(p);
	if (routePoints_.size() > 200)
		routePoints_.removeFirst();

	mapView_->setRoute(startPoint_, routePoints_);
}
