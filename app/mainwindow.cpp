#include "mainwindow.h"

#include <QDebug>
#include <QDoubleValidator>
#include <QFile>
#include <QGraphicsLayout>
#include <QIODevice>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QWebEngineSettings>
#include <QWebEngineView>
#include <QtCharts/QChartView>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include "./ui_mainwindow.h"
#include "QRangeSlider.hpp"

using json = nlohmann::json;

void MainWindow::enqueueStatusMessage(const QString& msg)
{
	statusQueue.enqueue(msg);

	if (!statusTimer->isActive())
	{
		showNextStatusMessage();
	}
}

void MainWindow::showNextStatusMessage()
{
	if (statusQueue.isEmpty())
	{
		ui->statusbar->clearMessage();
		return;
	}

	QString msg = statusQueue.dequeue();
	ui->statusbar->showMessage(msg, 3000);

	statusTimer->start(3000);
}

void MainWindow::sendToEsp(const QString& data)
{
	if (socket->state() == QAbstractSocket::ConnectedState)
	{
		socket->write((data + "\n").toUtf8());
		qDebug() << "Sent:" << data;
	}
}

void MainWindow::requestConfig()
{
	sendToEsp("GET_CONFIG");
}

void MainWindow::onReadyRead()
{
	while (socket->canReadLine())
	{
		QByteArray lineData = socket->readLine();
		QString line		= QString::fromUtf8(lineData).trimmed();

		if (line.isEmpty())
			continue;

		if (line.startsWith("SET "))
		{
			QStringList parts = line.split(" ");

			QString key = parts[1];
			float val	= parts[2].toFloat();

			firmwareParams[key] = val;

			QList<QLineEdit*> fields = findChildren<QLineEdit*>();
			for (QLineEdit* field : fields)
			{
				if (field->property("fw_key").toString() == key)
				{
					field->blockSignals(true);
					field->setText(QString::number(val));
					field->blockSignals(false);
					break;
				}
			}
		}
		else if (line == "START_CONFIG")
		{
			enqueueStatusMessage("The ESP32 has detected the request, and the "
								 "transmission is beginning");
		}
		else if (line == "END_CONFIG")
		{
			enqueueStatusMessage(
				"The ESP32 data has been successfully loaded into the GUI");
		}
		else if (line == "HI_GUI")
		{
			if (!isEspVerified)
			{
				isEspVerified = true;
				enqueueStatusMessage("Successful connection to ESP32");
				enqueueStatusMessage("Requesting data from the ESP32");
				requestConfig();
			}
		}
	}
}

void MainWindow::readFirmwareVars()
{
	QString appDir = QCoreApplication::applicationDirPath();
	QFile file(appDir + "/../../../airbag/data/firmware_vars.json");

	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		qDebug() << "Failed to open firmware_vars.json";
		return;
	}

	QByteArray jsonData = file.readAll();
	file.close();

	try
	{
		json j = json::parse(jsonData.toStdString());

		for (const auto& item : j)
		{
			for (auto const& [key, val] : item.items())
			{

				if (!val.contains("field_obj") || val["field_obj"].is_null())
				{
					continue;
				}

				std::string baseNameStd = val["field_obj"].get<std::string>();
				QString baseName		= QString::fromStdString(baseNameStd);

				float valFloat =
					val.contains("val") ? val["val"].get<float>() : NAN;
				QString valStr = QString::number(val["val"].get<double>());

				QString unitsStr = val.contains("units")
									   ? QString::fromStdString(
											 val["units"].get<std::string>())
									   : "";
				QString fieldNameStr =
					val.contains("field_name")
						? QString::fromStdString(
							  val["field_name"].get<std::string>())
						: "";
				QString infoStr =
					val.contains("info")
						? QString::fromStdString(val["info"].get<std::string>())
						: "";

				firmwareParams[QString::fromStdString(key)] = valFloat;

				QWidget* widget = this->findChild<QWidget*>(baseName);

				if (widget)
				{
					widget->setProperty("fw_key", QString::fromStdString(key));

					if (auto* lineEdit = qobject_cast<QLineEdit*>(widget))
					{
						lineEdit->setText(valStr);

						connect(lineEdit,
							&QLineEdit::editingFinished,
							this,
							[this, lineEdit]()
							{
								QString key =
									lineEdit->property("fw_key").toString();
								QString valStr = lineEdit->text();
								if (!valStr.isEmpty())
								{
									firmwareParams[key] = valStr.toFloat();
									sendToEsp(QString("SET %1 %2")
											.arg(key)
											.arg(valStr));
								}
							});
					}
					else if (auto* spinBox = qobject_cast<QSpinBox*>(widget))
					{
						spinBox->setValue(valStr.toInt());

						connect(spinBox,
							&QSpinBox::editingFinished,
							this,
							[this, spinBox]()
							{
								QString key =
									spinBox->property("fw_key").toString();
								int val				= spinBox->value();
								firmwareParams[key] = static_cast<float>(val);
								sendToEsp(
									QString("SET %1 %2").arg(key).arg(val));
							});
					}
					else if (auto* doubleSpinBox =
								 qobject_cast<QDoubleSpinBox*>(widget))
					{
						doubleSpinBox->setValue(valStr.toDouble());

						connect(doubleSpinBox,
							&QDoubleSpinBox::editingFinished,
							this,
							[this, doubleSpinBox]()
							{
								QString key = doubleSpinBox->property("fw_key")
												  .toString();
								double val			= doubleSpinBox->value();
								firmwareParams[key] = static_cast<float>(val);
								sendToEsp(
									QString("SET %1 %2").arg(key).arg(val));
							});
					}
				}

				QLabel* unitLabel = this->findChild<QLabel*>(baseName + "_u");
				if (unitLabel)
				{
					unitLabel->setText(unitsStr);
				}

				QLabel* nameLabel = this->findChild<QLabel*>(baseName + "_fn");
				if (nameLabel)
				{
					nameLabel->setText(fieldNameStr);
				}

				QLabel* infoLabel = this->findChild<QLabel*>(baseName + "_i");
				if (infoLabel)
				{
					infoLabel->setToolTip(infoStr);
				}
			}
		}
	}
	catch (json::parse_error& e)
	{
		qDebug() << "JSON parse error:" << e.what();
	}
}

void MainWindow::updateSeries(QChartView* view, int seriesIndex, float value)
{
	if (!view)
		return;

	QVariantList list = view->property("seriesList").toList();

	if (seriesIndex >= list.size())
		return;
	auto* series = list[seriesIndex].value<QLineSeries*>();

	QDateTime now = QDateTime::currentDateTime();
	series->append(now.toMSecsSinceEpoch(), value);

	qint64 window = 3 * 60 * 1000;
	qint64 cutoff = now.addMSecs(-window).toMSecsSinceEpoch();
	while (series->count() > 0 && series->at(0).x() < cutoff)
	{
		series->remove(0);
	}
}

void MainWindow::updateTempHumSeries()
{
	ui->front_temperature->setText(
		QString::number(firmwareParams["TEMP_FRONT"]));
	ui->rear_temperature->setText(QString::number(firmwareParams["TEMP_REAR"]));
	ui->front_humidity->setText(QString::number(firmwareParams["HUM_FRONT"]));
	ui->rear_humidity->setText(QString::number(firmwareParams["HUM_REAR"]));

	updateSeries(temperatureChartView, 0, firmwareParams["TEMP_FRONT"]);
	updateSeries(temperatureChartView, 1, firmwareParams["TEMP_REAR"]);

	updateSeries(humidityChartView, 0, firmwareParams["HUM_FRONT"]);
	updateSeries(humidityChartView, 1, firmwareParams["HUM_REAR"]);

	qint64 window = 4 * 60 * 1000;
	QDateTime now = QDateTime::currentDateTime();

	auto* axisX =
		temperatureChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);

	axisX = humidityChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);
}

void MainWindow::updateGizmo()
{
	sensorData.update(
		firmwareParams["PITCH"], firmwareParams["YAW"], firmwareParams["ROLL"]);

	ui->pitch->setText(QString::number(firmwareParams["PITCH"]));
	ui->yaw->setText(QString::number(firmwareParams["YAW"]));
	ui->roll->setText(QString::number(firmwareParams["ROLL"]));
}

void MainWindow::updateChannels()
{
	sensorData.update(
		firmwareParams["PITCH"], firmwareParams["YAW"], firmwareParams["ROLL"]);

	ui->pitch->setText(QString::number(firmwareParams["PITCH"]));
	ui->yaw->setText(QString::number(firmwareParams["YAW"]));
	ui->roll->setText(QString::number(firmwareParams["ROLL"]));
}

void MainWindow::updateMap()
{
	ui->gps_lat->setText(QString::number(firmwareParams["GPS_LAT"]));
	ui->gps_lon->setText(QString::number(firmwareParams["GPS_LON"]));
	ui->gps_spd->setText(QString::number(firmwareParams["GPS_SPD"]));
	ui->gps_sat->setText(QString::number(firmwareParams["GPS_SAT"]));

	ui->gps_day->setText(QString::number(firmwareParams["GPS_DAY"]));
	ui->gps_month->setText(QString::number(firmwareParams["GPS_MONTH"]));
	ui->gps_year->setText(QString::number(firmwareParams["GPS_YEAR"]));

	ui->gps_hour->setText(QString::number(firmwareParams["GPS_HOUR"]));
	ui->gps_min->setText(QString::number(firmwareParams["GPS_MIN"]));
	ui->gps_sec->setText(QString::number(firmwareParams["GPS_SEC"]));

	const QString js = QString("updatePosition(%1, %2);")
						   .arg(firmwareParams["GPS_LAT"], 0, 'f', 8)
						   .arg(firmwareParams["GPS_LON"], 0, 'f', 8);

	mapView->page()->runJavaScript(js);
}

void MainWindow::updateSeriesByTimer()
{
	updateSeries(pwmChartView, 0, firmwareParams["PWM_ME"]);
	updateSeries(pwmChartView, 1, firmwareParams["PWM_LE"]);
	updateSeries(pwmChartView, 2, firmwareParams["PWM_RE"]);

	ui->spinBox_me_pwm->setValue(firmwareParams["PWM_ME"]);
	ui->spinBox_le_pwm->setValue(firmwareParams["PWM_LE"]);
	ui->spinBox_re_pwm->setValue(firmwareParams["PWM_RE"]);

	updateSeries(deltaPWMChartView,
		0,
		firmwareParams["PWM_LE"] - firmwareParams["PWM_RE"]);

	updateSeries(accelerationChartView, 0, firmwareParams["ACCEL_X"]);
	updateSeries(accelerationChartView, 1, firmwareParams["ACCEL_Y"]);
	updateSeries(accelerationChartView, 2, firmwareParams["ACCEL_Z"]);

	updateSeries(angularVelocityChartView, 0, firmwareParams["ANG_VEL_X"]);
	updateSeries(angularVelocityChartView, 1, firmwareParams["ANG_VEL_Y"]);
	updateSeries(angularVelocityChartView, 2, firmwareParams["ANG_VEL_Z"]);

	qint64 window = 4 * 60 * 1000;
	QDateTime now = QDateTime::currentDateTime();

	auto* axisX = pwmChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);

	axisX = deltaPWMChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);

	axisX = accelerationChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);

	axisX = angularVelocityChartView->property("axisX").value<QDateTimeAxis*>();
	axisX->setRange(now.addMSecs(-window), now);

	updateTempHumSeries();
	updateGizmo();



	sliders_["me"]->setMarker(firmwareParams["CH_MAIN_ENGINE_VAL"]);
	sliders_["le"]->setMarker(firmwareParams["CH_FORWARD_VAL"]);
	sliders_["re"]->setMarker(firmwareParams["CH_FORWARD_VAL"]);
	sliders_["rle"]->setMarker(firmwareParams["CH_ROTATE_VAL"]);
	sliders_["rre"]->setMarker(firmwareParams["CH_ROTATE_VAL"]);
	sliders_["arm"]->setMarker(firmwareParams["CH_ARM_VAL"]);
	sliders_["hh"]->setMarker(firmwareParams["CH_HH_VAL"]);

	updateMap();
}

QChartView* MainWindow::createStyledChart(QWidget* widget,
	const QString& titleX,
	const QString& titleY,
	float minY,
	float maxY,
	const QStringList& seriesNames)
{
	auto* chart = new QChart();
	chart->setBackgroundBrush(Qt::NoBrush);

	QList<QColor> colors = { Qt::red,
		Qt::green,
		Qt::yellow,
		Qt::cyan,
		Qt::magenta,
		Qt::white,
		Qt::blue };

	QList<QLineSeries*> createdSeries;

	for (int i = 0; i < seriesNames.size(); ++i)
	{
		auto* series = new QLineSeries();
		series->setName(seriesNames[i]);

		series->setColor(colors[i % colors.size()]);

		chart->addSeries(series);
		createdSeries.append(series);
	}

	auto* axisX = new QDateTimeAxis();
	auto* axisY = new QValueAxis();

	axisX->setTitleText(titleX);
	axisX->setTitleBrush(QBrush(QColorConstants::Svg::orange));
	axisX->setFormat("HH:mm");
	axisX->setGridLineColor(QColor("#404040"));
	axisX->setLinePenColor(QColor("#404040"));
	axisX->setLabelsColor(QColorConstants::Svg::orange);
	axisX->setTickCount(4);
	QFont labelFont("Roboto", 8);
	axisX->setLabelsFont(labelFont);

	axisY->setTitleText(titleY);
	axisY->setTitleBrush(QBrush(QColorConstants::Svg::orange));
	axisY->setRange(minY, maxY);
	axisY->setGridLineColor(QColor("#404040"));
	axisY->setLinePenColor(QColor("#404040"));
	axisY->setLabelsColor(QColorConstants::Svg::orange);
	axisY->setLabelsFont(labelFont);
	axisY->setTickCount(5);

	chart->addAxis(axisX, Qt::AlignBottom);
	chart->addAxis(axisY, Qt::AlignLeft);

	for (auto* series : createdSeries)
	{
		series->attachAxis(axisX);
		series->attachAxis(axisY);
	}

	chart->setTitle("");
	chart->legend()->setVisible(true);
	chart->legend()->setLabelColor(Qt::white);
	chart->setMargins(QMargins(0, 0, 0, 0));
	chart->layout()->setContentsMargins(0, 0, 0, 0);
	chart->setBackgroundRoundness(0);

	auto* chartView = new QChartView(chart);
	chartView->setRenderHint(QPainter::Antialiasing);
	chartView->setStyleSheet("background: transparent;");
	chartView->setAttribute(Qt::WA_TranslucentBackground);

	chartView->setProperty("axisX", QVariant::fromValue(axisX));
	chartView->setProperty("axisY", QVariant::fromValue(axisY));

	QVariantList seriesVariants;
	for (auto* s : createdSeries)
	{
		seriesVariants.append(QVariant::fromValue(s));
	}
	chartView->setProperty("seriesList", seriesVariants);

	auto pwmChartLayout = new QVBoxLayout(widget);

	pwmChartLayout->addWidget(chartView);
	pwmChartLayout->setContentsMargins(0, 0, 0, 0);

	return chartView;
}

QRangeSlider* MainWindow::createSlider(
	QWidget* container, const int min, const int max)
{
	auto* slider = new QRangeSlider(ui->centralwidget);
	slider->setRange(min, max);
	QVBoxLayout* box = new QVBoxLayout(container);
	box->setContentsMargins(0, 0, 0, 0);
	box->addWidget(slider);

	return slider;
}

void MainWindow::setupRangeSliders()
{
	sliders_["me"] = createSlider(ui->rangeSlider_me_Container, 0, 2500);

	auto&& me_slider = sliders_["me"];

	me_slider->setLowValue(firmwareParams["CH_MAIN_ENGINE_MIN"]);
	me_slider->setHighValue(firmwareParams["CH_MAIN_ENGINE_MAX"]);
	me_slider->setMarker(firmwareParams["CH_MAIN_ENGINE_VAL"]);

	sliders_["le"] = createSlider(ui->rangeSlider_le_Container, 0, 2500);

	auto&& le_slider = sliders_["le"];

	le_slider->setLowValue(firmwareParams["CH_FORWARD_MIN"]);
	le_slider->setHighValue(firmwareParams["CH_FORWARD_MAX"]);
	le_slider->setMarker(firmwareParams["CH_FORWARD_VAL"]);

	sliders_["re"] = createSlider(ui->rangeSlider_re_Container, 0, 2500);

	auto&& re_slider = sliders_["re"];

	re_slider->setLowValue(firmwareParams["CH_FORWARD_MIN"]);
	re_slider->setHighValue(firmwareParams["CH_FORWARD_MAX"]);
	re_slider->setMarker(firmwareParams["CH_FORWARD_VAL"]);

	sliders_["rle"] = createSlider(ui->rangeSlider_rle_Container, 0, 2500);

	auto&& rle_slider = sliders_["rle"];

	rle_slider->setLowValue(firmwareParams["CH_ROTATE_MIN"]);
	rle_slider->setHighValue(firmwareParams["CH_ROTATE_MAX"]);
	rle_slider->setMarker(firmwareParams["CH_ROTATE_VAL"]);

	sliders_["rre"] = createSlider(ui->rangeSlider_rre_Container, 0, 2500);

	auto&& rre_slider = sliders_["rre"];

	rre_slider->setLowValue(firmwareParams["CH_ROTATE_MIN"]);
	rre_slider->setHighValue(firmwareParams["CH_ROTATE_MAX"]);
	rre_slider->setMarker(firmwareParams["CH_ROTATE_VAL"]);

	sliders_["arm"] = createSlider(ui->rangeSlider_arm_Container, 0, 2500);

	auto&& arm_slider = sliders_["arm"];

	arm_slider->setLowValue(firmwareParams["CH_ARM_MIN"]);
	arm_slider->setHighValue(firmwareParams["CH_ARM_MAX"]);
	arm_slider->setMarker(firmwareParams["CH_ARM_VAL"]);

	sliders_["hh"] = createSlider(ui->rangeSlider_hh_Container, 0, 2500);

	auto&& hh_slider = sliders_["hh"];

	hh_slider->setLowValue(firmwareParams["CH_HH_MIN"]);
	hh_slider->setHighValue(firmwareParams["CH_HH_MAX"]);
	hh_slider->setMarker(firmwareParams["CH_HH_VAL"]);

	ui->spinBox_re_channel->setValue(ui->spinBox_le_channel->value());
	ui->spinBox_rre_channel->setValue(ui->spinBox_rle_channel->value());
	connect(ui->spinBox_le_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui->spinBox_re_channel,
		&QSpinBox::setValue);

	connect(ui->spinBox_re_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui->spinBox_le_channel,
		&QSpinBox::setValue);

	connect(ui->spinBox_rle_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui->spinBox_rre_channel,
		&QSpinBox::setValue);

	connect(ui->spinBox_rre_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui->spinBox_rle_channel,
		&QSpinBox::setValue);

	ui->rangeSliders_scrollArea->setEnabled(ui->channels_checkBox->isChecked());

	auto* overlay = new QWidget(ui->rangeSliders_scrollArea);
	overlay->setStyleSheet("background-color: rgba(0, 0, 0, 150);");
	overlay->setGeometry(ui->rangeSliders_scrollArea->rect());
	overlay->raise();

	connect(ui->channels_checkBox,
		&QCheckBox::toggled,
		ui->rangeSliders_scrollArea,
		&QWidget::setEnabled);
	connect(ui->channels_checkBox,
		&QCheckBox::toggled,
		this,
		[this, overlay](bool checked)
		{
			ui->rangeSliders_scrollArea->setEnabled(checked);
			overlay->setVisible(!checked);
			overlay->raise();
		});
}
QString MainWindow::getWifiInterfaceName()
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

void MainWindow::WifiSsid()
{
	QString iface = getWifiInterfaceName();
	if (iface.isEmpty())
	{
		ui->wifi_field->setText("Not connected");
		return;
	}

	QProcess proc;
	proc.start(
		"nmcli", { "-t", "-f", "GENERAL.CONNECTION", "dev", "show", iface });
	proc.waitForFinished(2000);
	QString output = QString::fromUtf8(proc.readAllStandardOutput()).trimmed();
	QString ssid   = output.section(':', 1);
	ui->wifi_field->setText(
		ssid.isEmpty() || ssid == "--" ? "Not connected" : ssid);
}

MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->gridLayout_graphs->setRowStretch(0, 1);
	ui->gridLayout_graphs->setRowStretch(1, 1);
	ui->gridLayout_graphs->setColumnStretch(0, 1);
	ui->gridLayout_graphs->setColumnStretch(1, 1);

	qDebug() << "scrollArea viewport size:"
			 << ui->rangeSliders_scrollArea->viewport()->size();
	qDebug() << "scrollAreaWidgetContents size:"
			 << ui->scrollAreaWidgetContents->size();
	qDebug() << "scrollAreaWidgetContents sizeHint:"
			 << ui->scrollAreaWidgetContents->sizeHint();
	qDebug() << "scrollAreaWidgetContents layout:"
			 << ui->scrollAreaWidgetContents->layout();

	readFirmwareVars();

	statusTimer = new QTimer(this);
	statusTimer->setSingleShot(true);
	connect(statusTimer,
		&QTimer::timeout,
		this,
		&MainWindow::showNextStatusMessage);

	socket = new QTcpSocket(this);
	WifiSsid();
	QString ip	 = ui->ip_field->text();
	quint16 port = static_cast<quint16>(ui->port_field->text().toUInt());

	qDebug() << "IP: " << ui->ip_field->text() << "PORT: " << port;

	socket->connectToHost(ip, port);
	isEspVerified = false;

	connect(socket,
		&QTcpSocket::connected,
		this,
		[this]()
		{
			enqueueStatusMessage("TCP Connected. Verifying device...");

			sendToEsp("HI_ESP");
		});

	connect(socket, &QTcpSocket::readyRead, this, &MainWindow::onReadyRead);


	connect(ui->connection_button,
		&QPushButton::clicked,
		this,
		[this]()
		{
			if (!isEspVerified)
			{
				enqueueStatusMessage("Reconection...");
				WifiSsid();
				QString ip = ui->ip_field->text();
				quint16 port =
					static_cast<quint16>(ui->port_field->text().toUInt());
				socket->connectToHost(ip, port);

				sendToEsp("HI_ESP");
			}
			else
			{
				enqueueStatusMessage("esp32 already connected");
			}
		});

	pwmChartView = createStyledChart(ui->pwmChartContainer,
		"Time",
		"PWM, ms",
		990.0f,
		2010.0f,
		{ "Main Engine", "Left Engine", "Right Engine" });

	deltaPWMChartView = createStyledChart(ui->deltaPWMChartContainer,
		"Time",
		"PWM, ms",
		-1010,
		1010,
		{ "left - right PWM" });

	accelerationChartView = createStyledChart(ui->accelerationChartContainer,
		"Time",
		"Acceleration, m/s^2",
		-10,
		10,
		{ "X", "Y", "Z" });

	angularVelocityChartView =
		createStyledChart(ui->angularVelocityChartContainer,
			"Time",
			"Ang Velocity, deg/s",
			-60,
			60,
			{ "X", "Y", "Z" });

	temperatureChartView = createStyledChart(ui->temperatureChartContainer,
		"Time",
		"Temperature, C",
		-10,
		50,
		{ "Front", "Rear" });

	humidityChartView = createStyledChart(ui->humidityChartContainer,
		"Time",
		"Humidity, %",
		0,
		100,
		{ "Front", "Rear" });

	quickWidget = new QQuickWidget();
	quickWidget->setClearColor(Qt::transparent);
	quickWidget->setAttribute(Qt::WA_AlwaysStackOnTop);
	quickWidget->rootContext()->setContextProperty("sensorData", &sensorData);
	quickWidget->setSource(QUrl("../../Gizmo.qml"));

	QVBoxLayout* layout = new QVBoxLayout(ui->gizmoContainer);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(quickWidget);

	updatingSeriesTimer = new QTimer(this);
	updatingSeriesTimer->setInterval(100);
	connect(updatingSeriesTimer,
		&QTimer::timeout,
		this,
		&MainWindow::updateSeriesByTimer);
	updatingSeriesTimer->start();

	setupRangeSliders();


	mapView			 = new QWebEngineView(ui->gps_map);
	auto* map_layout = new QVBoxLayout(ui->gps_map);
	map_layout->setContentsMargins(0, 0, 0, 0);

	map_layout->addWidget(mapView);

	const QString tileServerPath =
		"/home/matvey/.nvm/versions/node/v25.8.1/bin/tileserver-gl";
	const QString mapDir  = "/home/matvey/work/Airbag/map";
	const QString mapPath = mapDir + "/central-fed-district-260818.mbtiles";

	tileServer = new QProcess(this);
	tileServer->setProgram(tileServerPath);
	tileServer->setArguments({ mapPath });
	tileServer->setProcessChannelMode(QProcess::MergedChannels);

	mapServer = new QProcess(this);
	mapServer->setProgram("python3");
	mapServer->setArguments({ "-m", "http.server", "8081" });
	mapServer->setWorkingDirectory(mapDir);
	mapServer->setProcessChannelMode(QProcess::MergedChannels);

	connect(tileServer,
		&QProcess::started,
		this,
		[this]() { qDebug() << "Tile server started"; });

	connect(mapServer,
		&QProcess::started,
		this,
		[this]()
		{
			qDebug() << "Map server started";
			QTimer::singleShot(500,
				mapView,
				[this]()
				{ mapView->load(QUrl("http://localhost:8081/map.html")); });
		});

	tileServer->start();
	mapServer->start();
}

MainWindow::~MainWindow()
{
	if (tileServer->state() != QProcess::NotRunning)
	{
		tileServer->terminate();

		if (!tileServer->waitForFinished(3000))
		{
			tileServer->kill();
		}
	}

	delete ui;
}
