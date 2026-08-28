#pragma once

#include <QHash>
#include <QObject>
#include <QQueue>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

class QStatusBar;
class QWidget;

// Owns the TCP link to the ESP32, the synced firmware parameters, and the
// status-bar message queue that reports connection progress.
class FirmwareLink : public QObject
{
	Q_OBJECT

public:
	explicit FirmwareLink(QStatusBar* statusBar, QObject* parent = nullptr);

	void connectToHost(const QString& ip, quint16 port);
void sendToEsp(const QString& key, float val);
void sendToEsp(const QString& command);
	void requestConfig();

	bool isVerified() const { return isEspVerified_; }

	void enqueueStatusMessage(const QString& msg);

signals:
void paramUpdated(const QString& key, float value);

private:
void onReadyRead();
void showNextStatusMessage();

	QTcpSocket* socket_;
	bool isEspVerified_ = false;

	QQueue<QString> statusQueue_;
	QTimer* statusTimer_;
	QStatusBar* statusBar_;

	QWidget* formRoot_ = nullptr;
};


