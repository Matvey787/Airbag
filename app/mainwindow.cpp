#include "mainwindow.h"

#include <QDebug>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) :
	QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	ui->gridLayout_graphs->setRowStretch(0, 1);
	ui->gridLayout_graphs->setRowStretch(1, 1);
	ui->gridLayout_graphs->setColumnStretch(0, 1);
	ui->gridLayout_graphs->setColumnStretch(1, 1);

	ui->horizontalLayout_gizmo->setStretch(0, 1);
	ui->horizontalLayout_gizmo->setStretch(1, 0);

	ui->verticalLayout_tempHumidity->setStretch(0, 1);
	ui->verticalLayout_tempHumidity->setStretch(1, 1);

	ui->horizontalLayout_gpsMap->setStretch(0, 1);
	ui->horizontalLayout_gpsMap->setStretch(1, 0);

	firmwareLink_ = new FirmwareLink(ui->statusbar, this);

	mapController_ = new MapController(ui->gps_map, this);

mapController_->addLabelBinding(ui->gps_lat, "GPS_LAT");
mapController_->addLabelBinding(ui->gps_lon, "GPS_LON");
mapController_->addLabelBinding(ui->gps_spd, "GPS_SPD");
mapController_->addLabelBinding(ui->gps_sat, "GPS_SAT");
mapController_->addLabelBinding(ui->gps_day, "GPS_DAY");
mapController_->addLabelBinding(ui->gps_month, "GPS_MONTH");
mapController_->addLabelBinding(ui->gps_year, "GPS_YEAR");
mapController_->addLabelBinding(ui->gps_hour, "GPS_HOUR");
mapController_->addLabelBinding(ui->gps_min, "GPS_MIN");
mapController_->addLabelBinding(ui->gps_sec, "GPS_SEC");


	mapController_->refreshWifiSsid(ui->wifi_field);

	QString ip	 = ui->ip_field->text();
	quint16 port = static_cast<quint16>(ui->port_field->text().toUInt());
	qDebug() << "IP: " << ip << "PORT: " << port;
	firmwareLink_->connectToHost(ip, port);

	connect(ui->connection_button,
		&QPushButton::clicked,
		this,
		[this]()
		{
			if (!firmwareLink_->isVerified())
			{
				firmwareLink_->enqueueStatusMessage("Reconection...");
				mapController_->refreshWifiSsid(ui->wifi_field);
				QString ip = ui->ip_field->text();
				quint16 port =
					static_cast<quint16>(ui->port_field->text().toUInt());
				firmwareLink_->connectToHost(ip, port);
				firmwareLink_->sendToEsp("HI_ESP");
			}
			else
			{
				firmwareLink_->enqueueStatusMessage("esp32 already connected");
			}
		});

firmwareHandler = new FirmwareVarsHandler(this);
firmwareHandler->readFirmwareVars(this);

connect(ui->save_button, &QPushButton::clicked, this, [this]() {
firmwareHandler->saveParamsToFile(this);
});
connect(ui->load_button, &QPushButton::clicked, this, [this]() {
firmwareHandler->loadParamsFromFile(this);
});

chartsHandler_ = new ChartsHandler(this);

chartsHandler_->createStyledChart(ui->pwmChartContainer,
		"PWM, ms",
		990.0f,
		2010.0f,
{ "PWM_ME", "PWM_LE", "PWM_RE" },
{ "Main Engine", "Left Engine", "Right Engine" });

// chartsHandler_.createStyledChart(ui->deltaPWMChartContainer,
// 	"PWM, ms",
// 	-1010,
// 	1010,
// 	{ "left - right PWM" },

// 	{ [](const QHash<QString, float>& p)
// 		{ return p.value("PWM_LE") - p.value("PWM_RE"); } });

 chartsHandler_->createStyledChart(ui->accelerationChartContainer,
		"Acceleration, m/s^2",
		-10,
		10,
{"ACCEL_X", "ACCEL_Y", "ACCEL_Z"
},
{ "X", "Y", "Z" });

chartsHandler_->createStyledChart(ui->angularVelocityChartContainer,
		"Ang Velocity, deg/s",
		-60,
		60,
{ "ANG_VEL_X", "ANG_VEL_Y", "ANG_VEL_Z"},
{ "X", "Y", "Z" });

chartsHandler_->createStyledChart(ui->temperatureChartContainer,
		"Temperature, C",
		-10,
		50,
{"TEMP_FRONT", "TEMP_REAR"},
{ "Front", "Rear" });

chartsHandler_->createStyledChart(ui->humidityChartContainer,
		"Humidity, %",
		0,
		100,
{ "HUM_FRONT", "HUM_REAR"},
{ "Front", "Rear" });
chartsHandler_->addLabelBinding(ui->front_temperature, "TEMP_FRONT");
chartsHandler_->addLabelBinding(ui->rear_temperature, "TEMP_REAR");
chartsHandler_->addLabelBinding(ui->front_humidity, "HUM_FRONT");
chartsHandler_->addLabelBinding(ui->rear_humidity, "HUM_REAR");

chartsHandler_->addLabelBinding(ui->label_me, "PWM_ME");
chartsHandler_->addLabelBinding(ui->label_le, "PWM_LE");
chartsHandler_->addLabelBinding(ui->label_re, "PWM_RE");

	gizmoController_ = new GizmoController(ui->gizmoContainer, this);

gizmoController_->addLabelBinding(ui->pitch, "PITCH");
gizmoController_->addLabelBinding(ui->yaw, "YAW");
gizmoController_->addLabelBinding(ui->roll, "ROLL");

	rangeSliders_ = new RangeSlidersPanel(ui, this);

connect(firmwareHandler, &FirmwareVarsHandler::paramUpdated,
rangeSliders_, &RangeSlidersPanel::onParamUpdated);

connect(firmwareHandler, &FirmwareVarsHandler::paramUpdatedSendToEsp,
firmwareLink_, qOverload<const QString&, float>(&FirmwareLink::sendToEsp));

connect(firmwareLink_, &FirmwareLink::paramUpdated,
firmwareHandler, &FirmwareVarsHandler::onParamUpdated);

	updatingSeriesTimer = new QTimer(this);
updatingSeriesTimer->setInterval(	100);
	connect(updatingSeriesTimer,
		&QTimer::timeout,
		this,
		&MainWindow::updateOnTimer);
	updatingSeriesTimer->start();
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::updateOnTimer()
{
	chartsHandler_->updateAll(firmwareHandler);

	gizmoController_->update(firmwareHandler);

	mapController_->updateMap(firmwareHandler);
}
