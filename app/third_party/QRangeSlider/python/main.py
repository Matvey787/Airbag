from PyQt5.QtWidgets import QApplication
from app.third_party.QRangeSlider.python.main_window import MainWindow

if __name__ == "__main__":
    app = QApplication([])

    window = MainWindow()
    window.show()

    app.exec()
