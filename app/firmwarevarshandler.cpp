#include "firmwarevarshandler.h"
#include "project_info.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QSpinBox>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QDir>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;


FirmwareVarsHandler::FirmwareVarsHandler(QObject* parent) : QObject(parent), formRoot_(nullptr) {}

void FirmwareVarsHandler::readFirmwareVars(QWidget* formRoot)
{
	formRoot_ = formRoot;
	firmwareParams_.clear();

	QFile file(firmwareParamsFilePath());
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

				firmwareParams_[QString::fromStdString(key)] = valFloat;

				QWidget* widget = formRoot->findChild<QWidget*>(baseName);

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
									firmwareParams_[key] = valStr.toFloat();
									emit paramUpdated(key, valStr.toFloat());
									emit paramUpdatedSendToEsp(key, valStr.toFloat());
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
								int val = spinBox->value();
								firmwareParams_[key] = static_cast<float>(val);
								emit paramUpdated(key, val);
								emit paramUpdatedSendToEsp(key, val);
							});
					}
					else if (auto* doubleSpinBox = qobject_cast<QDoubleSpinBox*>(widget))
					{
						doubleSpinBox->setValue(valStr.toDouble());

						connect(doubleSpinBox,
							&QDoubleSpinBox::editingFinished,
							this,
							[this, doubleSpinBox]()
							{
								QString key = doubleSpinBox->property("fw_key").toString();
								double val = doubleSpinBox->value();
								firmwareParams_[key] = static_cast<float>(val);
								emit paramUpdated(key, val);
								emit paramUpdatedSendToEsp(key, val);
							});
					}
				}

				emit paramUpdated(QString::fromStdString(key), valFloat);

				QLabel* unitLabel = formRoot->findChild<QLabel*>(baseName + "_u");
				if (unitLabel)
				{
					unitLabel->setText(unitsStr);
				}

				QLabel* nameLabel = formRoot->findChild<QLabel*>(baseName + "_fn");
				if (nameLabel)
				{
					nameLabel->setText(fieldNameStr);
				}

				QLabel* infoLabel = formRoot->findChild<QLabel*>(baseName + "_i");
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

void FirmwareVarsHandler::onParamUpdated(const QString& key, float val)
{
	firmwareParams_[key] = val;

qDebug() << key << ' ' << val;

	if (formRoot_)
	{
		const QList<QLineEdit*> lineEdits = formRoot_->findChildren<QLineEdit*>();
		for (QLineEdit* field : lineEdits)
		{
			if (field->property("fw_key").toString() == key)
			{
				field->blockSignals(true);
				field->setText(QString::number(val));
				field->blockSignals(false);
				emit paramUpdated(key, val);
				return;
			}
		}

		const QList<QSpinBox*> spinBoxes = formRoot_->findChildren<QSpinBox*>();
		for (QSpinBox* field : spinBoxes)
		{
			if (field->property("fw_key").toString() == key)
			{
				field->blockSignals(true);
				field->setValue(static_cast<int>(val));
				field->blockSignals(false);
				emit paramUpdated(key, val);
				return;
			}
		}

		const QList<QDoubleSpinBox*> doubleSpinBoxes = formRoot_->findChildren<QDoubleSpinBox*>();
		for (QDoubleSpinBox* field : doubleSpinBoxes)
		{
			if (field->property("fw_key").toString() == key)
			{
				field->blockSignals(true);
				field->setValue(static_cast<double>(val));
				field->blockSignals(false);
				emit paramUpdated(key, val);
				return;
			}
		}

		const QList<QLabel*> labels = formRoot_->findChildren<QLabel*>();
		for (QLabel* field : labels)
		{
			if (field->property("fw_key").toString() == key)
			{
				field->setText(QString::number(val));
				emit paramUpdated(key, val);
				return;
			}
		}
	}
emit paramUpdated(key, val);
}

const float FirmwareVarsHandler::operator[](const QString& key) const
{
	return firmwareParams_.value(key, 0.0f);
}

void FirmwareVarsHandler::saveParamsToFile(QWidget* parent)
{
	QString defaultFileName = QString("airbag_firmware_params_%1.json").arg(
		QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
	QString initialPath = QDir::homePath() + "/" + defaultFileName;

	QString fileName = QFileDialog::getSaveFileName(parent,
		"Save firmware params",
		initialPath,
		"JSON files (*.json);;All files (*)");
	if (fileName.isEmpty())
		return;

	json out;
	for (auto it = firmwareParams_.constBegin(); it != firmwareParams_.constEnd(); ++it)
	{
		out[ it.key().toStdString() ] = json::object({ {"val", it.value()} });
	}

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		QMessageBox::warning(parent, "Save failed", "Cannot open file for writing: " + fileName);
		return;
	}

	QByteArray text = QString::fromStdString(out.dump(4)).toUtf8();
	file.write(text);
	file.close();
}

void FirmwareVarsHandler::loadParamsFromFile(QWidget* parent)
{
	QString fileName = QFileDialog::getOpenFileName(parent,
		"Load firmware params",
		QString(),
		"JSON files (*.json);;All files (*)");
	if (fileName.isEmpty())
		return;

	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QMessageBox::warning(parent, "Load failed", "Cannot open file: " + fileName);
		return;
	}

	QByteArray jsonData = file.readAll();
	file.close();

	try
	{
		json j = json::parse(jsonData.toStdString());
		// Accept either an object mapping keys-> {"val": number} or an array of objects like original file
		if (j.is_object())
		{
for (auto it = j.begin(); it != j.end(); ++it)
{
	const std::string key = it.key();
	const auto& valNode = it.value();
	float val = 0.0f;
	if (valNode.is_object() && valNode.contains("val"))
	{
		val = valNode["val"].get<float>();
	}
	else if (valNode.is_number())
	{
		val = valNode.get<float>();
	}
	else
	{
		continue;
	}

	firmwareParams_[QString::fromStdString(key)] = val;
	onParamUpdated(QString::fromStdString(key), val);
}
		}
		else if (j.is_array())
		{
for (const auto& item : j)
{
	for (auto const& [key, val] : item.items())
	{
		float v = val.contains("val") ? val["val"].get<float>() : NAN;
		firmwareParams_[QString::fromStdString(key)] = v;
		onParamUpdated(QString::fromStdString(key), v);
	}
}
		}
	}
	catch (json::parse_error& e)
	{
		QMessageBox::warning(parent, "Parse error", "JSON parse error: " + QString::fromUtf8(e.what()));
		return;
	}
}
