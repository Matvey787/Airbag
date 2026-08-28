#include "QRangeSlider.hpp"

#include <QPainter>
#include <QPainterPath>
#include <stdexcept>

QRangeSlider::QRangeSlider(QWidget* parent) : QWidget(parent)
{
	lowValue_  = minimum_;
	highValue_ = maximum_;

	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

unsigned int QRangeSlider::minimum() const
{
	return minimum_;
}

unsigned int QRangeSlider::maximum() const
{
	return maximum_;
}

unsigned int QRangeSlider::lowValue() const
{
	return lowValue_;
}

unsigned int QRangeSlider::highValue() const
{
	return highValue_;
}

unsigned int QRangeSlider::step() const
{
	return step_;
}

void QRangeSlider::setStep(unsigned int step)
{
	step_ = step;
}

// Added orientation to support the promote option of a Horizontal Slider in QT
// designer.
void QRangeSlider::setOrientation(Qt::Orientation orientation)
{
	if (orientation != Qt::Horizontal)
		throw std::invalid_argument("Invalid Orientation. Horizontal is the "
									"only available orientation.");
}

void QRangeSlider::setBarDraggable(bool enable)
{
	barDraggable_ = enable;
}

QSize QRangeSlider::sizeHint() const
{
	return QSize(
		100 * HANDLE_SIZE + 2 * PADDING, 2 * HANDLE_SIZE + 2 * PADDING);
}

QSize QRangeSlider::minimumSizeHint() const
{
	return QSize(2 * HANDLE_SIZE + 2 * PADDING, 2 * HANDLE_SIZE);
}

void QRangeSlider::setMinimum(unsigned int minimum)
{
	if (minimum_ != minimum)
	{
		minimum_ = minimum;

		if (minimum_ >= maximum_)
		{
			setMaximum(minimum_ + 1);
			setLowValue(minimum_);
			setHighValue(maximum_);
		}
		else if (minimum_ >= highValue_)
		{
			setLowValue(minimum_);
			setHighValue(minimum_ + 1);
		}
		else if (minimum_ > lowValue_)
		{
			setLowValue(minimum_);
		}

		update();
		emit minimumChanged(minimum_);
		emit rangeChanged(minimum_, maximum_);
	}
}

void QRangeSlider::setMaximum(unsigned int maximum)
{
	if (maximum_ != maximum)
	{
		maximum_ = maximum;

		if (maximum_ <= minimum_)
		{
			setMinimum(maximum_ - 1);
			setLowValue(minimum_);
			setHighValue(maximum_);
		}
		else if (maximum_ <= lowValue_)
		{
			setLowValue(highValue_ - 1);
			setHighValue(maximum_);
		}
		else if (maximum_ < highValue_)
		{
			setHighValue(maximum_);
		}

		update();
		emit maximumChanged(maximum_);
		emit rangeChanged(minimum_, maximum_);
	}
}

void QRangeSlider::setRange(unsigned int minimum, unsigned int maximum)
{
	setMinimum(minimum);
	setMaximum(maximum);
}

void QRangeSlider::setLowValue(unsigned int lowValue)
{
	if (lowValue_ != lowValue)
	{
		lowValue_ = lowValue;

		if (lowValue_ >= maximum_)
			lowValue_ = maximum_ - 1;

		if (lowValue_ < minimum_)
			lowValue_ = minimum_;

		if (lowValue_ >= highValue_)
			setHighValue(lowValue_ + 1);

		update();
		emit lowValueChanged(lowValue_);
		emit valueChanged(lowValue_, highValue_);
	}
}

void QRangeSlider::setHighValue(unsigned int highValue)
{
	if (highValue_ != highValue)
	{
		highValue_ = highValue;

		if (highValue_ > maximum_)
			highValue_ = maximum_;

		if (highValue_ <= minimum_)
			highValue_ = minimum_ + 1;

		if (highValue_ <= lowValue_)
			setLowValue(highValue_ - 1);

		update();
		emit highValueChanged(highValue_);
		emit valueChanged(lowValue_, highValue_);
	}
}

void QRangeSlider::setValue(unsigned int lowValue, unsigned int highValue)
{
	setLowValue(lowValue);
	setHighValue(highValue);
}

void QRangeSlider::mousePressEvent(QMouseEvent* e)
{
	// Check if event was on slider.
	if (e->position().y() >= (height() - SLIDER_HEIGHT - HANDLE_SIZE) / 2 &&
		e->position().y() <= (height() - SLIDER_HEIGHT + HANDLE_SIZE) / 2)
	{
		double mouseX = e->position().x() < 0 ? 0 : e->position().x();
		unsigned int mouseValue =
			(mouseX / width()) * (maximum_ - minimum_) + minimum_;
		lastMouseValue_ = mouseValue;

		if (getLowHandleRect().contains(e->pos()))
			handleClicked_ = 0;
		else if (getHighHandleRect().contains(e->pos()))
			handleClicked_ = 1;
		else if (getRangeRect().contains(e->pos()))
			handleClicked_ = 2;
	}
}

void QRangeSlider::mouseReleaseEvent(QMouseEvent* e)
{
	Q_UNUSED(e);

	lastMouseValue_ = -1;
	handleClicked_	= -1;
}

void QRangeSlider::mouseMoveEvent(QMouseEvent* e)
{
	if (lastMouseValue_ != -1 && handleClicked_ != -1)
	{
		double mouseX = e->position().x() < 0 ? 0 : e->position().x();
		unsigned int mouseValue =
			(mouseX / width()) * (maximum_ - minimum_) + minimum_;

		if (handleClicked_ == 0)
		{
			setLowValue(mouseValue);
		}
		else if (handleClicked_ == 1)
		{
			setHighValue(mouseValue);
		}
		else if (barDraggable_ && handleClicked_ == 2)
		{
			int deltaValue = (mouseValue - lastMouseValue_);
			if (deltaValue < 0)
			{
				// Check for underflow
				setLowValue(lowValue_ + deltaValue > lowValue_
								? minimum_
								: lowValue_ + deltaValue);
				setHighValue(highValue_ + deltaValue);
			}
			else if (deltaValue > 0)
			{
				setLowValue(lowValue_ + deltaValue);
				// Check for overflow
				setHighValue(highValue_ + deltaValue < highValue_
								 ? maximum_
								 : highValue_ + deltaValue);
			}
		}

		lastMouseValue_ = mouseValue;
	}
}

double QRangeSlider::valueToX(unsigned int value) const
{
	double trackWidth = width() - 2.0 * PADDING;
	return PADDING + trackWidth * (value - minimum_) / (maximum_ - minimum_);
}

void QRangeSlider::drawRuler(QPainter& painter) const
{
	const int rulerHeight = 20;
	const int tickCount	  = 5;

	painter.setPen(QPen(Qt::GlobalColor::gray, 1));
	QFont font = painter.font();
	font.setPointSize(8);
	painter.setFont(font);

	for (int i = 0; i < tickCount; ++i)
	{
		unsigned int value =
			minimum_ + (maximum_ - minimum_) * i / (tickCount - 1);

		double x = valueToX(value);
		if (value == minimum_)
		{
			x += 5;
		}
		else if (value == maximum_)
		{
			x -= 12;
		}

		painter.drawLine(QPointF(x, rulerHeight - 6), QPointF(x, rulerHeight));

		QString label = QString::number(value);
		QRectF textRect(x - 15, 0, 30, rulerHeight - 6);
		painter.drawText(textRect, Qt::AlignCenter, label);
	}
}

void QRangeSlider::setMarker(const int val)
{
	markerVal_ = val;
update();
}

void QRangeSlider::drawMarker(QPainter& painter, const QColor& color) const
{

	double baseY = (height() - SLIDER_HEIGHT) / 2.0 - 2;

	int MARKER_SIZE = 12;

	QPainterPath path;

	double x = valueToX(markerVal_);

	path.moveTo(x, baseY);
	path.lineTo(x - MARKER_SIZE / 2.0, baseY - MARKER_SIZE);
	path.lineTo(x + MARKER_SIZE / 2.0, baseY - MARKER_SIZE);
	path.closeSubpath();

	painter.setPen(Qt::NoPen);
	painter.setBrush(QBrush(color));
	painter.drawPath(path);
}

void QRangeSlider::paintEvent(QPaintEvent* e)
{
	Q_UNUSED(e);

	QPainter painter(this);
	painter.setRenderHint(QPainter::RenderHint::Antialiasing);

	drawRuler(painter);
	drawMarker(painter, QColor(255, 165, 0));

	// Draw background
	painter.setPen(QPen(QColor(60, 60, 60), 0.8));
	painter.setBrush(QBrush(QColor(40, 40, 40)));
	painter.drawRoundedRect(getBackgroundRect(), 2, 2);

	// Draw range
	painter.setBrush(QBrush(QColor(255, 165, 0)));
	painter.drawRect(getRangeRect());

	// Draw lower handle
	painter.setBrush(QBrush(QColor(Qt::GlobalColor::white)));
	painter.drawRoundedRect(getLowHandleRect(), 2, 2);

	// Draw higher handle
	painter.drawRoundedRect(getHighHandleRect(), 2, 2);

	painter.end();
}

QRectF QRangeSlider::getBackgroundRect() const
{

	return QRectF(PADDING,
		(height() - SLIDER_HEIGHT) / 2,
		width() - 2 * PADDING,
		SLIDER_HEIGHT);
}

QRectF QRangeSlider::getRangeRect() const
{
	return QRectF(PADDING + ((width() - 2 * PADDING) * (lowValue_ - minimum_) /
								(maximum_ - minimum_)),
		(height() - SLIDER_HEIGHT) / 2,
		(width() - 2 * PADDING) * (highValue_ - lowValue_) /
			(maximum_ - minimum_),
		SLIDER_HEIGHT);
}

QRectF QRangeSlider::getLowHandleRect() const
{
	double trackWidth = width() - 2.0 * PADDING;
	double lowX =
		PADDING + trackWidth * (lowValue_ - minimum_) / (maximum_ - minimum_);
	double highX =
		PADDING + trackWidth * (highValue_ - minimum_) / (maximum_ - minimum_);
	double x = qMin(lowX, highX - HANDLE_SIZE);
	return QRectF(x, (height() - HANDLE_SIZE) / 2, HANDLE_SIZE, HANDLE_SIZE);
}

QRectF QRangeSlider::getHighHandleRect() const
{
	double trackWidth = width() - 2.0 * PADDING;
	double lowX =
		PADDING + trackWidth * (lowValue_ - minimum_) / (maximum_ - minimum_);
	double highX =
		PADDING + trackWidth * (highValue_ - minimum_) / (maximum_ - minimum_);
	double lowHandleX = qMin(lowX, highX - HANDLE_SIZE);
	double x		  = qMax(highX - HANDLE_SIZE, lowHandleX + HANDLE_SIZE);

	return QRectF(x, (height() - HANDLE_SIZE) / 2, HANDLE_SIZE, HANDLE_SIZE);
}
