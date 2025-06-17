from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import sys

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl('http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV2.html'))  # Assuming your Flask server is running on this address
        self.setCentralWidget(self.browser)
        self.showMaximized()

app = QApplication(sys.argv)
QApplication.setApplicationName('GPS For Dashboard')
window = MainWindow()
app.exec_()