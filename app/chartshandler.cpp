#include "chartshandler.h"
#include "firmwarevarshandler.h"

#include <QColor> // Вместо QColorConstants
#include <QDateTime>
#include <QGraphicsLayout>
#include <QPainter>
#include <QVBoxLayout>
#include <QtCharts/QDateTimeAxis>
#include <QtCharts/QValueAxis>

ChartsHandler::ChartsHandler(QObject* parent) : QObject(parent) {}

QChartView* ChartsHandler::createStyledChart(QWidget* parentWidget,
	const QString& titleY,
	float minY,
	float maxY,
	const QStringList& seriesKeys,
	const QStringList& seriesNames)
{
	if (!parentWidget->layout())
	{
		parentWidget->setLayout(new QVBoxLayout(parentWidget));
	}
	parentWidget->layout()->setContentsMargins(0, 0, 0, 0);
	parentWidget->layout()->setSpacing(0);

	auto* chart = new QChart();
	chart->setBackgroundBrush(Qt::NoBrush);
	chart->legend()->setLabelColor(Qt::white);

	chart->setMargins(QMargins(0, 0, 0, 0));
	chart->layout()->setContentsMargins(0, 0, 0, 0);

	const QList<QColor> colors = {
		Qt::red, Qt::green, Qt::yellow, Qt::cyan, Qt::magenta
	};
	QList<QLineSeries*> createdSeries;

	for (int i = 0; i < seriesKeys.size(); ++i)
	{
		auto* series = new QLineSeries();
		QString displayName =
			(i < seriesNames.size()) ? seriesNames[i] : seriesKeys[i];
		series->setName(displayName);
		series->setColor(colors[i % colors.size()]);
		chart->addSeries(series);
		createdSeries.append(series);
	}

	auto* axisX = new QDateTimeAxis();
	axisX->setFormat("HH:mm:ss");
	axisX->setTitleBrush(QBrush(QColor("orange")));
	axisX->setLabelsColor(QColor("orange"));
	axisX->setGridLineColor(QColor("#404040"));
	axisX->setTitleText("Time");

	auto* axisY = new QValueAxis();
	axisY->setTitleText(titleY);
	axisY->setRange(minY, maxY);
	axisY->setTitleBrush(QBrush(QColor("orange")));
	axisY->setLabelsColor(QColor("orange"));
	axisY->setGridLineColor(QColor("#404040"));

	chart->addAxis(axisX, Qt::AlignBottom);
	chart->addAxis(axisY, Qt::AlignLeft);

	for (auto* series : createdSeries)
	{
		series->attachAxis(axisX);
		series->attachAxis(axisY);
	}

	auto* chartView = new QChartView(chart, parentWidget);
	chartView->setRenderHint(QPainter::Antialiasing);
	chartView->setStyleSheet("background: transparent;");

	chartView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

	chartsList_.append({ chart, createdSeries, seriesKeys });

	parentWidget->layout()->addWidget(chartView);

	return chartView;
}

void ChartsHandler::appendAndTrim(QLineSeries* series, double x, float y)
{
	series->append(x, y);
	constexpr qint64 windowMs = 3 * 60 * 1000;
	while (series->count() > 0 && series->at(0).x() < x - windowMs)
	{
		series->remove(0);
	}
}

void ChartsHandler::addLabelBinding(QLabel* label, const QString& key)
{
labelBindings_.append({ label, key });
}

void ChartsHandler::updateAll(const FirmwareVarsHandler* firmwareParamsHandler)
{
	if (!firmwareParamsHandler)
	{
		return;
	}

	double nowMs	  = QDateTime::currentMSecsSinceEpoch();
	double rangeStart = nowMs - (180 * 1000);

	for (const auto& ctx : chartsList_)
	{
		for (int i = 0; i < ctx.series.size(); ++i)
		{
			if (i >= ctx.seriesKeys.size())
				break;

			const QString& key = ctx.seriesKeys[i];
			appendAndTrim(ctx.series[i], nowMs, (*firmwareParamsHandler)[key]);
		}

		auto axes = ctx.chart->axes(Qt::Horizontal);
		if (!axes.isEmpty())
		{
			if (auto* axisX = qobject_cast<QDateTimeAxis*>(axes.first()))
			{
				if (ctx.series[0]->count() > 0)
				{
					axisX->setRange(QDateTime::fromMSecsSinceEpoch(rangeStart),
						QDateTime::fromMSecsSinceEpoch(nowMs));
				}
			}
		}
	}

for (const auto& binding : labelBindings_)
{
	binding.label->setText(
		QString::number((*firmwareParamsHandler)[binding.key]));
}
}
