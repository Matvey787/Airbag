#include "rangesliderspanel.h"

#include <QCheckBox>
#include <QEvent>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>

#include "./ui_mainwindow.h"
#include "QRangeSlider.hpp"

RangeSlidersPanel::RangeSlidersPanel(Ui::MainWindow* ui, QObject* parent) :
	QObject(parent), ui_(ui)
{
	sliders_ = {
		{ "me",
			{ "CH_MAIN_ENGINE_VAL",
				"CH_MAIN_ENGINE_MIN",
				"CH_MAIN_ENGINE_MAX",
				createSlider(ui_->rangeSlider_me_Container, 0, 2500) } },
		{ "le",
			{ "CH_FORWARD_VAL",
				"CH_FORWARD_MIN",
				"CH_FORWARD_MAX",
				createSlider(ui_->rangeSlider_le_Container, 0, 2500) } },
		{ "re",
			{ "CH_FORWARD_VAL",
				"CH_FORWARD_MIN",
				"CH_FORWARD_MAX",
				createSlider(ui_->rangeSlider_re_Container, 0, 2500) } },
		{ "rle",
			{ "CH_ROTATE_VAL",
				"CH_ROTATE_MIN",
				"CH_ROTATE_MAX",
				createSlider(ui_->rangeSlider_rle_Container, 0, 2500) } },
		{ "rre",
			{ "CH_ROTATE_VAL",
				"CH_ROTATE_MIN",
				"CH_ROTATE_MAX",
				createSlider(ui_->rangeSlider_rre_Container, 0, 2500) } },
		{ "arm",
			{ "CH_ARM_VAL",
				"CH_ARM_MIN",
				"CH_ARM_MAX",
				createSlider(ui_->rangeSlider_arm_Container, 0, 2500) } },
		{ "hh",
			{ "CH_HH_VAL",
				"CH_HH_MIN",
				"CH_HH_MAX",
				createSlider(ui_->rangeSlider_hh_Container, 0, 2500) } }
	};

	ui_->spinBox_re_channel->setValue(ui_->spinBox_le_channel->value());
	ui_->spinBox_rre_channel->setValue(ui_->spinBox_rle_channel->value());

	QObject::connect(ui_->spinBox_le_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui_->spinBox_re_channel,
		&QSpinBox::setValue);
	QObject::connect(ui_->spinBox_re_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui_->spinBox_le_channel,
		&QSpinBox::setValue);
	QObject::connect(ui_->spinBox_rle_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui_->spinBox_rre_channel,
		&QSpinBox::setValue);
	QObject::connect(ui_->spinBox_rre_channel,
		QOverload<int>::of(&QSpinBox::valueChanged),
		ui_->spinBox_rle_channel,
		&QSpinBox::setValue);

	ui_->rangeSliders_scrollArea->setEnabled(
		ui_->channels_checkBox->isChecked());

	channelsOverlay_ = new QWidget(ui_->rangeSliders_scrollArea);
	channelsOverlay_->setStyleSheet("background-color: rgba(0, 0, 0, 150);");
	channelsOverlay_->setGeometry(ui_->rangeSliders_scrollArea->rect());
	channelsOverlay_->setVisible(!ui_->channels_checkBox->isChecked());
	channelsOverlay_->raise();
	ui_->rangeSliders_scrollArea->installEventFilter(this);

	QObject::connect(ui_->channels_checkBox,
		&QCheckBox::toggled,
		ui_->rangeSliders_scrollArea,
		&QWidget::setEnabled);
	QObject::connect(ui_->channels_checkBox,
		&QCheckBox::toggled,
		this,
		[this](bool checked)
		{
			ui_->rangeSliders_scrollArea->setEnabled(checked);
			channelsOverlay_->setVisible(!checked);
			channelsOverlay_->raise();
		});
}

QRangeSlider* RangeSlidersPanel::createSlider(
	QWidget* container, int min, int max)
{
	auto* slider = new QRangeSlider(ui_->centralwidget);
	slider->setRange(min, max);
	auto* box = new QVBoxLayout(container);
	box->setContentsMargins(0, 0, 0, 0);
	box->addWidget(slider);

	return slider;
}

void RangeSlidersPanel::onParamUpdated(const QString& key, float value)
{
	for (const auto& sl : sliders_)
	{
		if (sl.markerFirmVar == key)
		{
			sl.slider->setMarker(static_cast<int>(value));
		}
		else if (sl.minFirmVar == key)
		{
			sl.slider->setLowValue(static_cast<unsigned int>(value));
		}
		else if (sl.maxFirmVar == key)
		{
			sl.slider->setHighValue(static_cast<unsigned int>(value));
		}
	}
}

bool RangeSlidersPanel::eventFilter(QObject* watched, QEvent* event)
{
	if (watched == ui_->rangeSliders_scrollArea &&
		event->type() == QEvent::Resize && channelsOverlay_)
	{
		channelsOverlay_->setGeometry(ui_->rangeSliders_scrollArea->rect());
	}

	return QObject::eventFilter(watched, event);
}
