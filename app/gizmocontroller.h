#pragma once

#include "sensordata.h"
#include "firmwarevarshandler.h"

#include <QObject>
#include <QLabel>

class QQuickWidget;
class QWidget;

namespace Ui
{
class MainWindow;
}

// Owns the QML gizmo widget and the SensorData model it's bound to.
class GizmoController : public QObject
{
	Q_OBJECT

public:
	explicit GizmoController(QWidget* container, QObject* parent = nullptr);

void addLabelBinding(QLabel* label, const QString& key);
void update(const FirmwareVarsHandler& firmwareParamsHandler);

private:
	QQuickWidget* quickWidget_;
	SensorData sensorData_;

struct LabelBinding
{
QLabel* label;
QString key;
};

QList<LabelBinding> labelBindings_;
};
