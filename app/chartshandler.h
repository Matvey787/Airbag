#pragma once

#include "firmwarevarshandler.h"

#include <QHash>
#include <QList>
#include <QObject>
#include <QString>
#include <QLabel>
#include <functional>
#include <initializer_list>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

class ChartsHandler : public QObject
{
	Q_OBJECT

public:
	explicit ChartsHandler(QObject* parent = nullptr);

	QChartView* createStyledChart(QWidget* parentWidget,
		const QString& titleY,
		float minY,
		float maxY,
		const QStringList& seriesKeys,
		const QStringList& seriesNames);

QChartView* createStyledChart(QWidget* parentWidget,
	const QString& titleY,
	float minY,
	float maxY,
	std::initializer_list<QString> seriesNames,
	std::initializer_list<std::function<float(const QHash<QString, float>&)>>
		seriesFunctions);

void updateAll(const FirmwareVarsHandler* firmwareParamsHandler);
void addLabelBinding(QLabel* label, const QString& key);

private:
	struct ChartContext
	{
		QChart* chart;
		QList<QLineSeries*> series;
		QStringList seriesKeys;
		QList<std::function<float(const QHash<QString, float>&)>>
			seriesFunctions;
	};

	QList<ChartContext> chartsList_;

struct LabelBinding
{
QLabel* label;
QString key;
};

QList<LabelBinding> labelBindings_;

	static void appendAndTrim(QLineSeries* series, double x, float y);
};
