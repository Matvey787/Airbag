#pragma once

#include <QHash>
#include <QObject>
#include <QString>

class FirmwareVarsHandler : public QObject
{
	Q_OBJECT

private:
	QWidget* formRoot_;
	QHash<QString, float> firmwareParams_;

signals:
	void paramUpdated(const QString& key, float val);
	void paramUpdatedSendToEsp(const QString& key, float val);

public:
	FirmwareVarsHandler(QObject* parent);
	void onParamUpdated(const QString& key, float val);
	const float operator[](const QString& key) const;
	void readFirmwareVars(QWidget* formRoot);

	// Save current params (only key -> {"val": number}) to a JSON file chosen by the user
	void saveParamsToFile(QWidget* parent);

	// Load simplified JSON file and apply params (emit paramUpdated for each key)
	void loadParamsFromFile(QWidget* parent);
};
