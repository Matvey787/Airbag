#include "gizmocontroller.h"

#include <QQmlContext>
#include <QQuickWidget>
#include <QUrl>
#include <QVBoxLayout>

#include "firmwarevarshandler.h"

GizmoController::GizmoController(QWidget* container, QObject* parent) :
	QObject(parent)
{
	quickWidget_ = new QQuickWidget();
	quickWidget_->setResizeMode(QQuickWidget::SizeRootObjectToView);
	quickWidget_->setClearColor(Qt::transparent);
	quickWidget_->setAttribute(Qt::WA_AlwaysStackOnTop);
	quickWidget_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	quickWidget_->setMinimumSize(0, 0);
	quickWidget_->rootContext()->setContextProperty("sensorData", &sensorData_);
	quickWidget_->setSource(QUrl("../../Gizmo.qml"));

	auto* layout = new QVBoxLayout(container);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->addWidget(quickWidget_);
}


void GizmoController::addLabelBinding(QLabel* label, const QString& key)
{
	labelBindings_.append({ label, key });
}

void GizmoController::update(const FirmwareVarsHandler& firmwareParamsHandler)
{
	sensorData_.update(firmwareParamsHandler["PITCH"],
		firmwareParamsHandler["YAW"],
		firmwareParamsHandler["ROLL"]);

 qDebug() << "pitch =" << firmwareParamsHandler["PITCH"];

	for (const auto& binding : labelBindings_)
	{
		binding.label->setText(
			QString::number(firmwareParamsHandler[binding.key]));
	}
}
