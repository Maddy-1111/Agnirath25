# import sys
# from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
# from PyQt5.QtGui import QIcon, QPixmap


# class MyWindow(QMainWindow):
#     def __init__(self):
#         super().__init__()
        
        
#         button_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png")
#         self.button = QPushButton(self)
#         self.button.setIcon(QIcon("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"))  # Replace "icon_off.png" with the original icon file
#         self.button.setIconSize(button_image.size())
#         self.button.setGeometry(27, 140, button_image.width(), button_image.height())
#         self.button.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
#         # Create QIcon with multiple QPixmaps for different modes and states
#         icon = QIcon()
#         icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"), QIcon.Normal, QIcon.Off)  # Original icon
#         icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
#         self.button.setIcon(icon)

#         self.button.setCheckable(True)
#         self.button.clicked.connect(self.handle_button_click)

#     def handle_button_click(self):
#         pass


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     window = MyWindow()
#     window.setGeometry(100, 100, 200, 100)
#     window.show()
#     sys.exit(app.exec())

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QSlider, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtCore import Qt


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        button_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png")
        self.button = QPushButton(self)
        self.button.setIcon(QIcon("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"))  # Replace "icon_off.png" with the original icon file
        self.button.setIconSize(button_image.size())
        self.button.setGeometry(27, 140, button_image.width(), button_image.height())
        self.button.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        # Create QIcon with multiple QPixmaps for different modes and states
        icon = QIcon()
        icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"), QIcon.Normal, QIcon.Off)  # Original icon
        icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button.setIcon(icon)

        self.button.setCheckable(True)
        self.button.clicked.connect(self.handle_button_click)

        self.slider = QSlider(Qt.Vertical, self)
        self.slider.setGeometry(316,549,100,10)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        self.slider.setTickInterval(10)
        self.slider.valueChanged.connect(self.update_label)
        self.slider.hide()

        self.slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                border: 1px solid #999999;
                height: 8px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4);
                margin: 2px 0;
            }

            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #D5D5D5, stop:1 #E8E8E8);
                border: 1px solid #999999;
                width: 30px;
                margin: -2px 0;
                border-radius: 3px;
            }

            QSlider::sub-page:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #B1B1B1, stop:1 #c4c4c4);
            }
            """
        )

        self.label = QLabel("0", self)
        self.label.hide()

        layout = QVBoxLayout()
        layout.addWidget(self.button)
        layout.addWidget(self.slider)
        layout.addWidget(self.label)

        central_widget = QWidget(self)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

    def handle_button_click(self):
        if self.button.isChecked():
            # self.button.setIcon(QIcon("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"))  # Replace "icon_on.png" with the clicked icon file
            self.slider.show()
            self.label.show()
        else:
            # self.button.setIcon(QIcon("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"))  # Replace "icon_off.png" with the original icon file
            self.slider.hide()
            self.label.hide()
    def update_label(self, value):
        self.label.setText(str(value))

    # def keyPressEvent(self, event):
    #     if event.key() == Qt.Key_Escape:
    #         self.close()

   


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.setGeometry(100, 100, 200, 300)
    window.show()
    sys.exit(app.exec())
