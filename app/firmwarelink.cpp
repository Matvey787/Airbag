#include "firmwarelink.h"

#include <QDebug>
#include <QDoubleSpinBox>
#include <QIODevice>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QStatusBar>
#include <QRegularExpression>
#include <QWidget>



FirmwareLink::FirmwareLink(QStatusBar* statusBar, QObject* parent) :
	QObject(parent), socket_(new QTcpSocket(this)), statusBar_(statusBar)
{
	statusTimer_ = new QTimer(this);
	statusTimer_->setSingleShot(true);
	connect(
		statusTimer_, &QTimer::timeout, this, &FirmwareLink::showNextStatusMessage);

	connect(socket_, &QTcpSocket::readyRead, this, &FirmwareLink::onReadyRead);
	connect(socket_,
		&QTcpSocket::connected,
		this,
		[this]()
		{
			enqueueStatusMessage("TCP Connected. Verifying device...");
			sendToEsp("HI_ESP");
		});
}

void FirmwareLink::connectToHost(const QString& ip, quint16 port)
{
	isEspVerified_ = false;
	socket_->connectToHost(ip, port);
}

void FirmwareLink::sendToEsp(const QString& key, float val)
{
	if (socket_->state() == QAbstractSocket::ConnectedState)
	{
		const QString packet = QStringLiteral("SET %1 %2\n")
				.arg(key)
				.arg(val, 0, 'f', 3);
		socket_->write(packet.toUtf8());
	}
}

void FirmwareLink::sendToEsp(const QString& command)
{
if (socket_->state() == QAbstractSocket::ConnectedState)
{
socket_->write((command + "\n").toUtf8());
}
}

void FirmwareLink::requestConfig()
{
	sendToEsp("GET_CONFIG");
}

void FirmwareLink::enqueueStatusMessage(const QString& msg)
{
	statusQueue_.enqueue(msg);

	if (!statusTimer_->isActive())
	{
		showNextStatusMessage();
	}
}

void FirmwareLink::showNextStatusMessage()
{
	if (statusQueue_.isEmpty())
	{
		statusBar_->clearMessage();
		return;
	}

	QString msg = statusQueue_.dequeue();
	statusBar_->showMessage(msg, 3000);

	statusTimer_->start(3000);
}

void FirmwareLink::onReadyRead()
{
	while (socket_->canReadLine())
	{
		QByteArray lineData = socket_->readLine();
		QString line		= QString::fromUtf8(lineData).trimmed();

		if (line.isEmpty())
			continue;

		if (line.startsWith("SET "))
		{
QStringList parts = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
if (parts.size() < 3)
{
	qDebug() << "Malformed SET packet:" << line;
	continue;
}

QString key = parts[1];
float val = parts[2].toFloat();
emit paramUpdated(key, val);
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
			if (!isEspVerified_)
			{
				isEspVerified_ = true;
				enqueueStatusMessage("Successful connection to ESP32");
				enqueueStatusMessage("Requesting data from the ESP32");
requestConfig();
			}
		}
	}
}

