from typing import override

from PyQt5.QtCore import pyqtSignal, pyqtSlot, QSize, QRectF
from PyQt5.QtWidgets import QWidget, QSizePolicy
from PyQt5.QtGui import QPainter, QColorConstants, QPen, QBrush, QColor, QMouseEvent, QPaintEvent

class QRangeSlider(QWidget):

    # Minimum range value
    _minimum = 0

    # Maximum range value
    _maximum = 100

    # Step value
    _step = 1

    # Value of the last mouse click
    _lastMouseValue = -1

    # Whether the slider bar can be drag
    _barDraggable = False

    # Handle of the last mouse click. 0 is low handle, 1 is high handle and 2 is slider bar
    _handleClicked = -1

    # Painter constants
    # Slider height in pixels
    SLIDER_HEIGHT = 5
    # Slider handle size in pixels
    HANDLE_SIZE = 13
    # Left and Right padding in pixels
    PADDING = 1

    # Signals
    minimumChange = pyqtSignal(int)
    maximumChange = pyqtSignal(int)
    lowValueChange = pyqtSignal(int)
    highValueChange = pyqtSignal(int)
    rangeChange = pyqtSignal(int, int)

    def __init__(self, parent = None):
        super().__init__(parent)

        # Value of low handle
        self._lowValue = self._minimum

        # Value of high handle
        self._highValue = self._maximum

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def minimum(self):
        return self._minimum

    @pyqtSlot(int)
    def setMinimum(self, minimum):
        if self._minimum != minimum:
            self._minimum = minimum
            if self._minimum >= self._maximum:
                self.setMaximum(self._minimum + 1)
                self.setHighValue(self._maximum)
                self.setLowValue(self._minimum)
            elif self._minimum >= self._highValue:
                self.setHighValue(self._minimum + 1)
                self.setLowValue(self._minimum)
            elif self._minimum > self._lowValue:
                self.setLowValue(self._minimum)

            self.update()
            self.minimumChange.emit(self._minimum)
            self.rangeChange.emit(self._minimum, self._maximum)

    def maximum(self):
        return self._maximum

    @pyqtSlot(int)
    def setMaximum(self, maximum):
        if self._maximum != maximum:
            self._maximum = maximum
            if self._maximum <= self._minimum:
                self.setMinimum(self._maximum - 1)
                self.setHighValue(self._maximum)
                self.setLowValue(self._minimum)
            elif self._maximum <= self._lowValue:
                self.setHighValue(self._maximum)
                self.setLowValue(self._highValue -1)
            elif self._maximum < self._highValue:
                self.setHighValue(self._maximum)

            self.update()
            self.maximumChange.emit(self._maximum)
            self.rangeChange.emit(self._minimum, self._maximum)

    def lowValue(self):
        return self._lowValue

    @pyqtSlot(int)
    def setLowValue(self, lowValue):
        if self._lowValue != lowValue:
            self._lowValue = lowValue
            if self._lowValue >= self._maximum:
                self._lowValue = self._maximum - 1
            if self._lowValue < self._minimum:
                self._lowValue = self._minimum
            if self._lowValue >= self._highValue:
                self.setHighValue(self._lowValue + 1)

            self.update()
            self.lowValueChange.emit(self._lowValue)

    def highValue(self):
        return self._highValue

    @pyqtSlot(int)
    def setHighValue(self, highValue):
        if self._highValue != highValue:
            self._highValue = highValue
            if self._highValue > self._maximum:
                self._highValue = self._maximum
            if self._highValue <= self._minimum:
                self._highValue = self._minimum + 1
            if self._highValue <= self._lowValue:
                self.setLowValue(self._highValue - 1)

            self.update()
            self.highValueChange.emit(self._highValue)

    def step(self):
        return self._step

    def setStep(self, step):
        self._step = step

    @pyqtSlot(int, int)
    def setRange(self, minimum, maximum):
        self.setMinimum(minimum)
        self.setMaximum(maximum)

    def sizeHint(self):
        return QSize(100 * self.HANDLE_SIZE + 2 * self.PADDING, 2 * self.HANDLE_SIZE + 2 * self.PADDING)

    def minimumSizeHint(self):
        return QSize(2 * self.HANDLE_SIZE + 2 * self.PADDING, 2 * self.HANDLE_SIZE)

    def setBarDraggable(self, enable):
        self._barDraggable = enable

    def mousePressEvent(self, a0: QMouseEvent | None):
        if a0 is None:
            return
        # Check if event was on slider
        if a0.pos().y() >= (self.height() - self.SLIDER_HEIGHT - self.HANDLE_SIZE) / 2 and a0.pos().y() <= (self.height() - self.SLIDER_HEIGHT + self.HANDLE_SIZE) / 2:
            mouseX = 0 if a0.pos().x() < 0 else a0.pos().x()
            mouseValue = int((mouseX / self.width()) * (self._maximum - self._minimum) + self._minimum)
            self._lastMouseValue = mouseValue

            if (self.getLowHandleRect().contains(a0.pos())):
                self._handleClicked = 0
            if (self.getHighHandleRect().contains(a0.pos())):
                self._handleClicked = 1
            if (self.getRangeRect().contains(a0.pos())):
                self._handleClicked = 2

    def mouseReleaseEvent(self, a0: QMouseEvent | None):
        self._lastMouseValue = -1
        self._handleClicked = -1

    def mouseMoveEvent(self, a0: QMouseEvent | None):
        if a0 is None:
            return

        if self._lastMouseValue != -1 and self._handleClicked != -1:
            mouseX = 0 if a0.pos().x() < 0 else a0.pos().x()
            mouseValue = int((mouseX / self.width()) * (self._maximum - self._minimum) + self._minimum)

            if self._handleClicked == 0:
                self.setLowValue(mouseValue)
            elif self._handleClicked == 1:
                self.setHighValue(mouseValue)
            elif self._handleClicked == 2:
                deltaValue = (mouseValue - self._lastMouseValue)
                self.setLowValue(self._lowValue + deltaValue)
                self.setHighValue(self._highValue + deltaValue)

            self._lastMouseValue = mouseValue

    def paintEvent(self, a0: QPaintEvent | None):
        if a0 is None:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw background
        painter.setPen(QPen(QColorConstants.DarkGray, 0.8))
        painter.setBrush(QBrush(QColor(QColorConstants.LightGray)))
        painter.drawRoundedRect(self.getBackgroundRect(), 2, 2)

        # Draw range
        painter.setBrush(QBrush(QColor(0x1E, 0x90, 0xFF)))
        painter.drawRect(self.getRangeRect())

        # Draw lower handle
        painter.setBrush(QBrush(QColor(QColorConstants.White)))
        painter.drawRoundedRect(self.getLowHandleRect(), 2, 2)

        # Draw higher handle
        painter.drawRoundedRect(self.getHighHandleRect(), 2, 2)

        painter.end()

    def getBackgroundRect(self):
        return QRectF(
            self.PADDING,
            (self.height() - self.SLIDER_HEIGHT) / 2,
            self.width() - 2 * self.PADDING,
            self.SLIDER_HEIGHT)

    def getRangeRect(self):
        return QRectF(
            self.PADDING + ((self.width() - 2 * self.PADDING) * (self._lowValue - self._minimum) / (self._maximum - self._minimum)),
            (self.height() - self.SLIDER_HEIGHT) / 2,
            (self.width() - 2 * self.PADDING) * (self._highValue - self._lowValue) / (self._maximum - self._minimum),
            self.SLIDER_HEIGHT)

    def getLowHandleRect(self):
        trackWidth = self.width() - 2.0 * self.PADDING
        lowX = self.PADDING + trackWidth * (self._lowValue - self._minimum) / (self._maximum - self._minimum)
        highX = self.PADDING + trackWidth * (self._highValue - self._minimum) / (self._maximum - self._minimum)
        x = min(lowX, highX - self.HANDLE_SIZE)
        return QRectF(x, (self.height() - self.HANDLE_SIZE) / 2, self.HANDLE_SIZE, self.HANDLE_SIZE)

    def getHighHandleRect(self):
        trackWidth = self.width() - 2.0 * self.PADDING
        lowX = self.PADDING + trackWidth * (self._lowValue - self._minimum) / (self._maximum - self._minimum)
        highX = self.PADDING + trackWidth * (self._highValue - self._minimum) / (self._maximum - self._minimum)
        lowHandleX = min(lowX, highX - self.HANDLE_SIZE)
        x = max(highX - self.HANDLE_SIZE, lowHandleX + self.HANDLE_SIZE)
        return QRectF(x, (self.height() - self.HANDLE_SIZE) / 2, self.HANDLE_SIZE, self.HANDLE_SIZE)
