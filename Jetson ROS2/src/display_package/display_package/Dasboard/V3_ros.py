import sys
import cv2
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QSlider
from PyQt5.QtGui import QPixmap, QIcon, QImage, QPalette, QBrush, QPainter, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QRect, QUrl, QTimer
from PyQt5.QtWebEngineWidgets import QWebEngineView
import random
from array import array
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray

class RosThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self.node)

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.topic_name = '/final_data'
        self.connect_ros()

        # Create a QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_random_data)
        self.timer.start(1000)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.setFixedSize(1024, 600)

        pixmap = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Bg.png")
        self.central_widget.setAutoFillBackground(True)
        palette = self.central_widget.palette()
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.central_widget.setPalette(palette)

        self.camera_label = QLabel(self)
        self.camera_label.setGeometry(QRect(185, 27, 806 , 510))
        self.cam = Camera()

        self.dist_label = QLabel(self)
        self.dist_label.setGeometry(QRect(250, 495, 245, 25))
        self.dist_label.setAlignment(Qt.AlignCenter)
        self.dist_label.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.dist_label.setText("DISTANCE")
        self.dist_label.hide()

        self.dist_ul = QLabel(self)
        self.dist_ul.setGeometry(250,525, 245, 9)
        self.dist_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(245, 9))
        self.dist_ul.hide()        

        self.solar1 = QLabel(self)
        self.solar1.setGeometry(QRect(218, 57, 200, 36))
        self.solar1.setAlignment(Qt.AlignCenter)
        self.solar1.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.solar1.setText("COCKPIT")
        self.solar1.hide()

        self.solar1_ul = QLabel(self)
        self.solar1_ul.setGeometry(212, 100 , 220, 9)
        self.solar1_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.solar1_ul.hide()

        self.solar3 = QLabel(self)
        self.solar3.setGeometry(QRect(744,57,200,36))
        self.solar3.setAlignment(Qt.AlignCenter)
        self.solar3.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.solar3.setText("MOTOR")
        self.solar3.hide()

        self.solar3_ul = QLabel(self)
        self.solar3_ul.setGeometry(744,100, 220, 9)
        self.solar3_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.solar3_ul.hide()

        self.solar4 = QLabel(self)
        self.solar4.setGeometry(QRect(218, 337, 450, 36))
        self.solar4.setAlignment(Qt.AlignCenter)
        self.solar4.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.solar4.setText("MOTOR CONTROLLER")
        self.solar4.hide()

        self.solar4_ul = QLabel(self)
        self.solar4_ul.setGeometry(218,376, 450, 9)
        self.solar4_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(450, 9))
        self.solar4_ul.hide()

        self.mppt_temp = QLabel(self)
        self.mppt_temp.setGeometry(QRect(754, 337, 200, 36))
        self.mppt_temp.setAlignment(Qt.AlignCenter)
        self.mppt_temp.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.mppt_temp.setText("BATTERY")
        self.mppt_temp.hide()

        self.mppt_temp_ul = QLabel(self)
        self.mppt_temp_ul.setGeometry(744,380, 220, 9)
        self.mppt_temp_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.mppt_temp_ul.hide()

        self.motor_current = QLabel(self)
        self.motor_current.setGeometry(QRect(430, 50, 200, 36))
        self.motor_current.setAlignment(Qt.AlignCenter)
        self.motor_current.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.motor_current.setText("MOTOR")
        self.motor_current.hide()

        self.motor_current_ul = QLabel(self)
        self.motor_current_ul.setGeometry(430,90, 200, 9)
        self.motor_current_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.motor_current_ul.hide()

        self.motor_current_fr = QLabel(self)
        self.motor_current_fr.setGeometry(350,130, 173, 63)
        self.motor_current_fr.hide()

        self.motor_voltage_fr = QLabel(self)
        self.motor_voltage_fr.setGeometry(550,130, 173, 63)
        self.motor_voltage_fr.hide()

        self.motorctrl_current = QLabel(self)
        self.motorctrl_current.setGeometry(QRect(310, 220, 450, 36))
        self.motorctrl_current.setAlignment(Qt.AlignCenter)
        self.motorctrl_current.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.motorctrl_current.setText("MOTOR CONTROLLER")
        self.motorctrl_current.hide()

        self.motorctrl_current_ul = QLabel(self)
        self.motorctrl_current_ul.setGeometry(310,260, 450, 9)
        self.motorctrl_current_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(450, 9))
        self.motorctrl_current_ul.hide()

        self.motorctrl_current_fr = QLabel(self)
        self.motorctrl_current_fr.setGeometry(350,300, 173, 63)
        self.motorctrl_current_fr.hide()

        self.motorctrl_voltage_fr = QLabel(self)
        self.motorctrl_voltage_fr.setGeometry(550,300, 173, 63)
        self.motorctrl_voltage_fr.hide()

        self.battery_current = QLabel(self)
        self.battery_current.setGeometry(QRect(430, 390, 200, 36))
        self.battery_current.setAlignment(Qt.AlignCenter)
        self.battery_current.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.battery_current.setText("BATTERY")
        self.battery_current.hide()

        self.battery_current_ul = QLabel(self)
        self.battery_current_ul.setGeometry(430,430, 200, 9)
        self.battery_current_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.battery_current_ul.hide()

        self.battery_current_fr = QLabel(self)
        self.battery_current_fr.setGeometry(350,470, 173, 63)
        self.battery_current_fr.hide()

        self.battery_voltage_fr = QLabel(self)
        self.battery_voltage_fr.setGeometry(550,470, 173, 63)
        self.battery_voltage_fr.hide()

        self.time_label = QLabel(self)
        self.time_label.setGeometry(QRect(687, 495, 250, 25))
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setStyleSheet("font-family: 'Good Times'; font-size: 32px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.time_label.setText("DRIVE TIME")
        self.time_label.hide()

        self.time_ul = QLabel(self)
        self.time_ul.setGeometry(687,525, 250, 9)
        self.time_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(250, 9))
        self.time_ul.hide()

        self.mapfr = QLabel(self)
        self.mapfr.setGeometry(185, 27 , 806, 455)
        self.mapfr.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Map display frame.png").scaled(806, 455, Qt.AspectRatioMode.KeepAspectRatio))
        self.mapfr.hide()

        self.distfr = QLabel(self)
        self.distfr.setGeometry(290, 536, 175, 46)
        self.distfr.hide()

        self.timefr = QLabel(self)
        self.timefr.setGeometry(722, 536, 175, 46)
        self.timefr.hide()

        self.ctrlfr = QLabel(self)
        self.ctrlfr.setGeometry(185, 18 , 806, 355)
        self.ctrlfr.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/control frame.png").scaled(806, 355, Qt.AspectRatioMode.KeepAspectRatio))
        self.ctrlfr.hide()

        self.fan_label = QLabel(self)
        self.fan_label.setGeometry(QRect(461, 28, 200, 65))
        self.fan_label.setAlignment(Qt.AlignCenter)
        self.fan_label.setStyleSheet("font-family: 'Good Times'; font-size: 60px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.fan_label.setText("FANS")
        self.fan_label.hide()    

        self.fan_ul = QLabel(self)
        self.fan_ul.setGeometry(450,90, 220, 9)
        self.fan_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(220, 9))
        self.fan_ul.hide() 

        self.fan1_label = QLabel(self)
        self.fan1_label.setGeometry(QRect(240, 145, 180, 60))
        self.fan1_label.setAlignment(Qt.AlignCenter)
        self.fan1_label.setStyleSheet("font-family: 'Good Times'; font-size: 45px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.fan1_label.setText("FAN 1")
        self.fan1_label.hide()

        self.fan2_label = QLabel(self)
        self.fan2_label.setGeometry(QRect(489, 145, 180, 60))
        self.fan2_label.setAlignment(Qt.AlignCenter)
        self.fan2_label.setStyleSheet("font-family: 'Good Times'; font-size: 45px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.fan2_label.setText("FAN 2")
        self.fan2_label.hide()

        self.fan3_label = QLabel(self)
        self.fan3_label.setGeometry(QRect(738, 145, 180, 60))
        self.fan3_label.setAlignment(Qt.AlignCenter)
        self.fan3_label.setStyleSheet("font-family: 'Good Times'; font-size: 45px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.fan3_label.setText("FAN 3")
        self.fan3_label.hide()

        self.slider1_label = QLabel("0", self)
        self.slider1_label.setGeometry(316,550,25,25)
        self.slider1_label.setAlignment(Qt.AlignCenter)
        self.slider1_label.setStyleSheet("font-family: 'Arial'; font-size: 15px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        "background-color: #161F28")
        self.slider1_label.hide() 

        self.slider2_label = QLabel("0", self)
        self.slider2_label.setGeometry(559,550,25,25)
        self.slider2_label.setAlignment(Qt.AlignCenter)
        self.slider2_label.setStyleSheet("font-family: 'Arial'; font-size: 15px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        "background-color: #161F28")
        self.slider2_label.hide()

        self.slider3_label = QLabel("0", self)
        self.slider3_label.setGeometry(811,550,25,25)
        self.slider3_label.setAlignment(Qt.AlignCenter)
        self.slider3_label.setStyleSheet("font-family: 'Arial'; font-size: 15px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #FFFFFF; "
                                        "background-color: #161F28")
        self.slider3_label.hide()  

        self.light_label = QLabel(self)
        self.light_label.setGeometry(QRect(432, 28, 300, 65))
        self.light_label.setAlignment(Qt.AlignCenter)
        self.light_label.setStyleSheet("font-family: 'Good Times'; font-size: 60px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.light_label.setText("LIGHTS")
        self.light_label.hide()

        self.light_ul = QLabel(self)
        self.light_ul.setGeometry(432,90, 300, 9)
        self.light_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(300, 9))
        self.light_ul.hide()

        self.sequence_label = QLabel(self)
        self.sequence_label.setGeometry(QRect(195, 384, 300, 65))
        self.sequence_label.setAlignment(Qt.AlignCenter)
        self.sequence_label.setStyleSheet("font-family: 'Good Times'; font-size: 40px; font-weight: 400; "
                                        "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                                        "background-color: #161F28")
        self.sequence_label.setText("SEQUENCE")
        self.sequence_label.hide() 

        self.state_label = QLabel(self)
        self.state_label.setGeometry(QRect(460, 28, 230, 65))
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-family: 'Good Times'; font-size: 60px; font-weight: 400; "
                              "line-height: 48px; letter-spacing: 0em; color: #F97110; "
                              "background-color: #161F28")
        self.state_label.setText("STATE")
        self.state_label.hide()

        self.state_ul = QLabel(self)
        self.state_ul.setGeometry(450,90, 250, 9)
        self.state_ul.setPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/underline.png").scaled(250, 9))
        self.state_ul.hide() 
        
        button1_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/voltage.png")
        button2_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Temprature.png")
        button3_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/current.png")
        button4_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/map.png")
        button5_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Controls.png")
        button6_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/fans.png")
        button7_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Lights.png")
        button8_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/State.png")
        button9_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/TBD.png")
        manual_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png")
        light1_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light1.png")
        light2_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light2.png")
        light3_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light3.png")
        light4_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light4.png")
        light5_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light5.png")
        light6_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light6.png")
        seq1_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq1.png")
        seq2_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq2.png")
        seq3_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq3.png")
        on_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/on.png")
        off_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/off.png")
        safe_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/safe.png")
        drive_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/drive.png")
        
        self.button1 = QPushButton(self)
        self.button1.setIcon(QIcon(button1_image))
        self.button1.setIconSize(button1_image.size())
        self.button1.setGeometry(27, 24, int(button1_image.width()), int(button1_image.height()))
        self.button1.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button1.clicked.connect(self.Mainwindow)

        self.button2 = QPushButton(self)
        self.button2.setIcon(QIcon(button2_image))
        self.button2.setIconSize(button2_image.size())
        self.button2.setGeometry(27, 140, button2_image.width(), button2_image.height())
        self.button2.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button2.clicked.connect(self.Temps)

        self.button3 = QPushButton(self)
        self.button3.setIcon(QIcon(button3_image))
        self.button3.setIconSize(button3_image.size())
        self.button3.setGeometry(27, 256, button3_image.width(), button3_image.height())
        self.button3.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button3.clicked.connect(self.Current)

        self.button4 = QPushButton(self)
        self.button4.setIcon(QIcon(button4_image))
        self.button4.setIconSize(button4_image.size())
        self.button4.setGeometry(27, 372, button4_image.width(), button4_image.height())
        self.button4.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button4.clicked.connect(self.Map)

        self.button5 = QPushButton(self)
        self.button5.setIcon(QIcon(button5_image))
        self.button5.setIconSize(button5_image.size())
        self.button5.setGeometry(27, 490, button5_image.width(), button5_image.height())
        self.button5.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button5.clicked.connect(self.CPanel)

        self.button6 = QPushButton(self)
        self.button6.setIcon(QIcon(button6_image))
        self.button6.setIconSize(button6_image.size())
        self.button6.setGeometry(273, 383, button6_image.width(), button6_image.height())
        self.button6.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button6.clicked.connect(self.Fans)
        self.button6.hide()

        self.button7 = QPushButton(self)
        self.button7.setIcon(QIcon(button7_image))
        self.button7.setIconSize(button7_image.size())
        self.button7.setGeometry(689, 383, button7_image.width(), button7_image.height())
        self.button7.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button7.clicked.connect(self.Lights)
        self.button7.hide()

        self.button8 = QPushButton(self)
        self.button8.setIcon(QIcon(button8_image))
        self.button8.setIconSize(button8_image.size())
        self.button8.setGeometry(273, 499, button8_image.width(), button8_image.height())
        self.button8.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button8.clicked.connect(self.State)
        self.button8.hide()

        self.button9 = QPushButton(self)
        self.button9.setIcon(QIcon(button9_image))
        self.button9.setIconSize(button9_image.size())
        self.button9.setGeometry(689, 499, button9_image.width(), button9_image.height())
        self.button9.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        self.button9.clicked.connect(self.TBD)
        self.button9.hide()

        self.button10 = QPushButton(self)
        self.button10.setIcon(QIcon(manual_image))
        self.button10.setIconSize(manual_image.size())
        self.button10.setGeometry(239, 221, manual_image.width(), manual_image.height())
        self.button10.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        manual_icon = QIcon()
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"), QIcon.Normal, QIcon.Off)  # Original icon
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button10.setIcon(manual_icon)
        self.button10.setCheckable(True)
        self.button10.clicked.connect(self.manual_button1_click)
        self.button10.hide()

        self.button11 = QPushButton(self)
        self.button11.setIcon(QIcon(manual_image))
        self.button11.setIconSize(manual_image.size())
        self.button11.setGeometry(489, 221, manual_image.width(), manual_image.height())
        self.button11.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        manual_icon = QIcon()
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"), QIcon.Normal, QIcon.Off)  # Original icon
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button11.setIcon(manual_icon)
        self.button11.setCheckable(True)
        self.button11.clicked.connect(self.manual_button2_click)
        self.button11.hide()

        self.button12 = QPushButton(self)
        self.button12.setIcon(QIcon(manual_image))
        self.button12.setIconSize(manual_image.size())
        self.button12.setGeometry(736, 221, manual_image.width(), manual_image.height())
        self.button12.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        manual_icon = QIcon()
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual.png"), QIcon.Normal, QIcon.Off)  # Original icon
        manual_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/manual_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button12.setIcon(manual_icon)
        self.button12.setCheckable(True)
        self.button12.clicked.connect(self.manual_button3_click)
        self.button12.hide()

        self.button13 = QPushButton(self)
        self.button13.setIcon(QIcon(light1_image))
        self.button13.setIconSize(light1_image.size())
        self.button13.setGeometry(261, 142, light1_image.width(), light1_image.height())
        self.button13.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light1_icon = QIcon()
        light1_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light1.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light1_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light1_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button13.setIcon(light1_icon)
        self.button13.setCheckable(True)
        self.button13.clicked.connect(self.light_button_click)
        self.button13.hide()

        self.button14 = QPushButton(self)
        self.button14.setIcon(QIcon(light2_image))
        self.button14.setIconSize(light2_image.size())
        self.button14.setGeometry(514, 142, light2_image.width(), light2_image.height())
        self.button14.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light2_icon = QIcon()
        light2_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light2.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light2_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light2_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button14.setIcon(light2_icon)
        self.button14.setCheckable(True)
        self.button14.clicked.connect(self.light_button_click)
        self.button14.hide()

        self.button15 = QPushButton(self)
        self.button15.setIcon(QIcon(light3_image))
        self.button15.setIconSize(light3_image.size())
        self.button15.setGeometry(784, 142, light3_image.width(), light3_image.height())
        self.button15.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light3_icon = QIcon()
        light3_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light3.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light3_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light3_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button15.setIcon(light3_icon)
        self.button15.setCheckable(True)
        self.button15.clicked.connect(self.light_button_click)
        self.button15.hide()

        self.button16 = QPushButton(self)
        self.button16.setIcon(QIcon(light4_image))
        self.button16.setIconSize(light4_image.size())
        self.button16.setGeometry(261, 272, light4_image.width(), light4_image.height())
        self.button16.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light4_icon = QIcon()
        light4_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light4.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light4_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light4_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button16.setIcon(light4_icon)
        self.button16.setCheckable(True)
        self.button16.clicked.connect(self.light_button_click)
        self.button16.hide()

        self.button17 = QPushButton(self)
        self.button17.setIcon(QIcon(light5_image))
        self.button17.setIconSize(light5_image.size())
        self.button17.setGeometry(514, 272, light5_image.width(), light5_image.height())
        self.button17.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light5_icon = QIcon()
        light5_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light5.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light5_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light5_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button17.setIcon(light5_icon)
        self.button17.setCheckable(True)
        self.button17.clicked.connect(self.light_button_click)
        self.button17.hide()

        self.button18 = QPushButton(self)
        self.button18.setIcon(QIcon(light6_image))
        self.button18.setIconSize(light6_image.size())
        self.button18.setGeometry(788, 272, light6_image.width(), light6_image.height())
        self.button18.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        light6_icon = QIcon()
        light6_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light6.png"), QIcon.Normal, QIcon.Off)  # Original icon
        light6_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Light6_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button18.setIcon(light6_icon)
        self.button18.setCheckable(True)
        self.button18.clicked.connect(self.light_button_click)
        self.button18.hide()

        self.button19 = QPushButton(self)
        self.button19.setIcon(QIcon(seq1_image))
        self.button19.setIconSize(seq1_image.size())
        self.button19.setGeometry(302, 466, seq1_image.width(), seq1_image.height())
        self.button19.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        seq1_icon = QIcon()
        seq1_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq1.png"), QIcon.Normal, QIcon.Off)  # Original icon
        seq1_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq1_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button19.setIcon(seq1_icon)
        self.button19.setCheckable(True)
        self.button19.clicked.connect(self.seq_button_click)
        self.button19.hide()

        self.button20 = QPushButton(self)
        self.button20.setIcon(QIcon(seq2_image))
        self.button20.setIconSize(seq2_image.size())
        self.button20.setGeometry(546, 466, seq2_image.width(), seq2_image.height())
        self.button20.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        seq2_icon = QIcon()
        seq2_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq2.png"), QIcon.Normal, QIcon.Off)  # Original icon
        seq2_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq2_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button20.setIcon(seq2_icon)
        self.button20.setCheckable(True)
        self.button20.clicked.connect(self.seq_button_click)
        self.button20.hide()

        self.button21 = QPushButton(self)
        self.button21.setIcon(QIcon(seq3_image))
        self.button21.setIconSize(seq3_image.size())
        self.button21.setGeometry(816, 466, seq3_image.width(), seq3_image.height())
        self.button21.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        seq3_icon = QIcon()
        seq3_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq3.png"), QIcon.Normal, QIcon.Off)  # Original icon
        seq3_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Seq3_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button21.setIcon(seq3_icon)
        self.button21.setCheckable(True)
        self.button21.clicked.connect(self.seq_button_click)
        self.button21.hide()

        self.button22 = QPushButton(self)
        self.button22.setIcon(QIcon(on_image))
        self.button22.setIconSize(on_image.size())
        self.button22.setGeometry(255, 154, on_image.width(), on_image.height())
        self.button22.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        on_icon = QIcon()
        on_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/on.png"), QIcon.Normal, QIcon.Off)  # Original icon
        on_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/on_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button22.setIcon(on_icon)
        self.button22.setCheckable(True)
        self.button22.clicked.connect(self.state_button_click)
        self.button22.hide()

        self.button23 = QPushButton(self)
        self.button23.setIcon(QIcon(safe_image))
        self.button23.setIconSize(safe_image.size())
        self.button23.setGeometry(679, 154, safe_image.width(), safe_image.height())
        self.button23.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        safe_icon = QIcon()
        safe_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/safe.png"), QIcon.Normal, QIcon.Off)  # Original icon
        safe_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/safe_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button23.setIcon(safe_icon)
        self.button23.setCheckable(True)
        self.button23.clicked.connect(self.state_button_click)
        self.button23.hide()

        self.button24 = QPushButton(self)
        self.button24.setIcon(QIcon(off_image))
        self.button24.setIconSize(off_image.size())
        self.button24.setGeometry(255, 377, off_image.width(), off_image.height())
        self.button24.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        off_icon = QIcon()
        off_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/off.png"), QIcon.Normal, QIcon.Off)  # Original icon
        off_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/off_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button24.setIcon(off_icon)
        self.button24.setCheckable(True)
        self.button24.clicked.connect(self.state_button_click)
        self.button24.hide()

        self.button25 = QPushButton(self)
        self.button25.setIcon(QIcon(drive_image))
        self.button25.setIconSize(drive_image.size())
        self.button25.setGeometry(679, 377, drive_image.width(), drive_image.height())
        self.button25.setStyleSheet("QPushButton { border: none; background-color: transparent; }")
        drive_icon = QIcon()
        drive_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/drive.png"), QIcon.Normal, QIcon.Off)  # Original icon
        drive_icon.addPixmap(QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/drive_clicked.png"), QIcon.Normal, QIcon.On)  # Clicked icon
        self.button25.setIcon(drive_icon)
        self.button25.setCheckable(True)
        self.button25.clicked.connect(self.state_button_click)
        self.button25.hide()

        self.slider1 = QSlider(Qt.Vertical, self)
        self.slider1.setGeometry(316,300,25,250)
        self.slider1.setMinimum(0)
        self.slider1.setMaximum(100)
        self.slider1.setTickPosition(QSlider.TicksBothSides)
        self.slider1.setTickInterval(10)
        self.slider1.valueChanged.connect(self.update_slider1)
        self.slider1.hide()

        self.slider2 = QSlider(Qt.Vertical, self)
        self.slider2.setGeometry(559,300,25,250)
        self.slider2.setMinimum(0)
        self.slider2.setMaximum(100)
        self.slider2.setTickPosition(QSlider.TicksBothSides)
        self.slider2.setTickInterval(10)
        self.slider2.valueChanged.connect(self.update_slider2)
        self.slider2.hide()

        self.slider3 = QSlider(Qt.Vertical, self)
        self.slider3.setGeometry(811,300,25,250)
        self.slider3.setMinimum(0)
        self.slider3.setMaximum(100)
        self.slider3.setTickPosition(QSlider.TicksBothSides)
        self.slider3.setTickInterval(10)
        self.slider3.valueChanged.connect(self.update_slider3)
        self.slider3.hide()

        self.temps = 0
        self.distance = 0
        self.motorcurrent_value = 0
        self.mtrctrl_value = 0
        self.batcurrent_value = 0
        self.time_str = "00:00:00"
        self.seconds = 0
        self.minutes = 0
        self.hours = 0

        self.temp_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Tempcool.png',
                            '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/tempok.png',
                            '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/temphot.png'
                            ]
        
        self.battery_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/battery_indicator.png',
                            '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/battery_indicator_red.png'
                            ]
        
        self.motor_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/motor_indicator.png',
                             '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/motor_indicator_red.png'
                            ]
        
        self.motorcntrl_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/motorcntrl_indicator.png',
                                  '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/motorcntrl_indicator_red.png'
                            ]
        
        self.mppt_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/mppt_indicator.png',
                            '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/mppt_indicator_red.png'
                            ]
        
        self.solar_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/solar_indicator.png',
                             '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/solar_indicator_red.png'
                            ]
        
        self.lv_images = ['/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/LV_indicator.png',
                          '/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/LV_indicator_red.png'
                            ]

        self.camera_label.show()
        self.cam.ImageUpdate.connect(self.ImageUpdateSlot)
        self.cam.start()
        self.load_html_file("http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV2.html")

        self.battery_label = QLabel(self)
        self.battery_label.setGeometry(848,347,143,216)

        self.temp1_label = QLabel(self)
        self.temp1_label.setGeometry(286,138,71,157)
        self.temp1_label.hide()
        self.temp2_label = QLabel(self)
        self.temp2_label.setGeometry(818,138,71,157)
        self.temp2_label.hide()
        self.temp3_label = QLabel(self)
        self.temp3_label.setGeometry(286,410,71,157)
        self.temp3_label.hide()
        self.temp4_label = QLabel(self)
        self.temp4_label.setGeometry(818,410,71,157)
        self.temp4_label.hide()

        self.battery_indicator_label = QLabel(self)
        self.battery_indicator_label.setGeometry(206,546,73,40)
        self.battery_indicator_label.hide()
        self.motor_indicator_label = QLabel(self)
        self.motor_indicator_label.setGeometry(342,546,73,40)
        self.motor_indicator_label.hide()
        self.motorcntrl_indicator_label = QLabel(self)
        self.motorcntrl_indicator_label.setGeometry(478,546,73,40)
        self.motorcntrl_indicator_label.hide()
        self.mppt_indicator_label = QLabel(self)
        self.mppt_indicator_label.setGeometry(610,546,73,40)
        self.mppt_indicator_label.hide()
        self.solar_indicator_label = QLabel(self)
        self.solar_indicator_label.setGeometry(746,546,73,40)
        self.solar_indicator_label.hide()
        self.lv_indicator_label = QLabel(self)
        self.lv_indicator_label.setGeometry(882,546,73,40)
        self.lv_indicator_label.hide()        

        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.update_variable)
        self.timer1.start(1000) 

        self.button1_clicked = False
        self.button2_clicked = False
        self.button3_clicked = False
        self.button4_clicked = False
        self.button5_clicked = False
        self.button6_clicked = False
        self.button7_clicked = False
        self.button8_clicked = False
        self.button9_clicked = False
        
    def connect_ros(self):
        try:
            # ROS2 init
            rclpy.init(args=None)
            self.node = Node('display_node')
            self.sub = self.node.create_subscription(
                rosarray,
                self.topic_name,
                self.sub_rosarray_callback,
                10,
            )        
            # Create a publisher
            self.pub = self.node.create_publisher(
                rosarray,
                '/control_data',
                10
            )
            
            # spin once, timeout_sec 5[s]
            timeout_sec_rclpy = 5
            timeout_init = time.time()
            rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
            timeout_end = time.time()
            ros_connect_time = timeout_end - timeout_init

            # Error Handle for rclpy timeout
            if ros_connect_time >= timeout_sec_rclpy:
                print("Couldn't Connect")
                # self.ui.label_ros2_state_float.setStyleSheet(
                #     "color: rgb(255,255,255);"
                #     "background-color: rgb(255,0,51);"
                #     "border-radius:5px;"
                # )
            else:
                print("Connected")
                self.ros_thread = RosThread(self.node)   # Create ros thread 
                self.ros_thread.start() 
                # self.ui.label_ros2_state_float.setStyleSheet(
                #     "color: rgb(255,255,255);"
                #     "background-color: rgb(18,230,95);"
                #     "border-radius:5px;"
                # )

        except:
            pass

    def publish_data(self, data):
        msg = rosarray()
        msg.data = data
        self.pub.publish(msg)

    def publish_random_data(self):
        # Generate random data
        data = array('f', [random.random()])
        print(data)
        self.publish_data(data)
        
        
    ### ROS2 Data Updater
    def sub_rosarray_callback(self, msg):
        self.data = msg.data
        print(self.data)
        # self.update_float_data_label()
    
    def hide(self):
        if self.button1_clicked:
            self.camera_label.show()
            self.cam.ImageUpdate.connect(self.ImageUpdateSlot)
            self.cam.start()
            self.camera_label.setGeometry(QRect(185, 27, 806 , 510))
            self.battery_label.show()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.motor_current.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.motor_current_ul.hide()
            self.mppt_temp_ul.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.show()
            self.motor_indicator_label.show()
            self.motorcntrl_indicator_label.show()
            self.mppt_indicator_label.show()
            self.solar_indicator_label.show()
            self.lv_indicator_label.show()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide()
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide() 
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide()
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide() 
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide() 
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()
            
        if self.button2_clicked:
            self.solar1.show()
            self.solar3.show()
            self.solar4.show()
            self.mppt_temp.show()
            self.solar1_ul.show()
            self.solar3_ul.show()
            self.solar4_ul.show()
            self.mppt_temp_ul.show()
            self.temp1_label.show()
            self.temp2_label.show()
            self.temp3_label.show()
            self.temp4_label.show()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide()
            self.fan2_label.hide()
            self.fan3_label.hide() 
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide()
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide()
            self.button13.hide()
            self.button14.hide()
            self.button16.hide()
            self.button15.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()

        if self.button3_clicked:
            self.motor_current.show()
            self.motor_current_ul.show()
            self.motor_current_fr.show()
            self.motor_voltage_fr.show()
            self.motorctrl_current.show()
            self.motorctrl_current_ul.show()
            self.motorctrl_current_fr.show()
            self.battery_current.show()
            self.battery_current_ul.show()
            self.battery_current_fr.show()
            self.battery_voltage_fr.show()
            self.motorctrl_voltage_fr.show()
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide()  
            self.fan_ul.hide() 
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide()
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide() 
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide()
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide() 

        if self.button4_clicked:
            self.dist_label.show()
            self.dist_ul.show()  
            self.time_label.show()
            self.timefr.show()
            self.time_ul.show()
            self.distfr.show()
            self.mapfr.show()
            self.web_view.show()
            self.camera_label.hide()
            self.cam.stop()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide()
            self.fan_ul.hide()
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide() 
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide()  
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide() 
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()

        if self.button5_clicked:
            self.ctrlfr.show()
            self.button6.show()
            self.button7.show()
            self.button8.show()
            self.button9.show()
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide() 
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide() 
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide()
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide() 

        if self.button6_clicked:
            self.fan_label.show()  
            self.fan_ul.show() 
            self.fan1_label.show()  
            self.fan2_label.show()
            self.fan3_label.show()  
            self.button10.show()
            self.button11.show()
            self.button12.show()
            self.slider3.hide()
            self.slider3_label.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.slider1.hide()
            self.slider1_label.hide() 
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide() 
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()
             
        if self.button7_clicked:
            self.sequence_label.show()   
            self.light_label.show() 
            self.light_ul.show() 
            self.button13.show()
            self.button14.show()
            self.button15.show()
            self.button16.show()
            self.button17.show()
            self.button18.show()
            self.button19.show()
            self.button20.show()
            self.button21.show()
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide() 
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()

        if self.button8_clicked:
            self.state_ul.show() 
            self.state_label.show() 
            self.button22.show()
            self.button23.show()
            self.button24.show()
            self.button25.show()
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.ctrlfr.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide() 
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide()
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()            

        if self.button9_clicked:
            self.camera_label.hide()
            self.cam.stop()
            self.dist_label.hide()
            self.time_label.hide()
            self.timefr.hide()
            self.distfr.hide()
            self.mapfr.hide()
            self.web_view.hide()
            self.battery_label.hide()
            self.solar1.hide()
            self.solar3.hide()
            self.solar4.hide()
            self.mppt_temp.hide()
            self.solar1_ul.hide()
            self.solar3_ul.hide()
            self.solar4_ul.hide()
            self.mppt_temp_ul.hide()
            self.temp1_label.hide()
            self.temp2_label.hide()
            self.temp3_label.hide()
            self.temp4_label.hide()
            self.battery_indicator_label.hide()
            self.motor_indicator_label.hide()
            self.motorcntrl_indicator_label.hide()
            self.mppt_indicator_label.hide()
            self.solar_indicator_label.hide()
            self.lv_indicator_label.hide()
            self.motor_current.hide()
            self.motor_current_ul.hide()
            self.motor_current_fr.hide()
            self.motor_voltage_fr.hide()
            self.motorctrl_current.hide()
            self.motorctrl_current_ul.hide()
            self.motorctrl_current_fr.hide()
            self.battery_current.hide()
            self.battery_current_ul.hide()
            self.battery_current_fr.hide()
            self.battery_voltage_fr.hide()
            self.motorctrl_voltage_fr.hide()
            self.dist_ul.hide()  
            self.time_ul.hide()
            self.button6.hide()
            self.button7.hide()
            self.button8.hide()
            self.button9.hide()
            self.fan_label.hide() 
            self.fan_ul.hide()  
            self.fan1_label.hide() 
            self.fan2_label.hide()
            self.fan3_label.hide()
            self.button10.hide()
            self.slider1.hide()
            self.slider1_label.hide() 
            self.button11.hide()
            self.slider2.hide()
            self.slider2_label.hide() 
            self.button12.hide()
            self.slider3.hide()
            self.slider3_label.hide()
            self.sequence_label.hide()   
            self.light_label.hide() 
            self.light_ul.hide()
            self.button13.hide()
            self.button14.hide()
            self.button15.hide()
            self.button16.hide()
            self.button17.hide()
            self.button18.hide()
            self.button19.hide()
            self.button20.hide()
            self.button21.hide()
            self.state_ul.hide() 
            self.state_label.hide() 
            self.button22.hide()
            self.button23.hide()
            self.button24.hide()
            self.button25.hide()

    def update_variable(self):
        self.temps += 1
        self.distance += 1
        self.motorcurrent_value += 1
        self.mtrctrl_value += 1
        self.batcurrent_value += 1
        self.display_image()
        self.seconds += 1
        if self.seconds == 60:
            self.seconds = 0
            self.minutes += 1
            if self.minutes == 60:
                self.minutes = 0
                self.hours += 1
        self.time_str = f"{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}"
        if self.temps > 80:
            self.temps = 0 
        if self.distance > 3050:
            self.distance = 0
        if self.motorcurrent_value > 20:
            self.motorcurrent_value = 0
        if self.mtrctrl_value > 25:
            self.mtrctrl_value = 0
        if self.batcurrent_value > 30:
            self.batcurrent_value = 0

    def display_image(self):
        # Determine the index for the indicator image
        battery_index = 0
        if self.data[0] == 1:
            battery_index = 1
        else:
            battery_index = 0

        motor_index = 0
        if self.data[1] == 1:
            motor_index = 1
        else:
            motor_index = 0

        motorcntrl_index = 0
        if self.data[2] == 1:
            motorcntrl_index = 1
        else:
            motorcntrl_index = 0

        mppt_index = 0
        if self.data[3] == 1:
            mppt_index = 1
        else:
            mppt_index = 0

        solar_index = 0
        if self.data[4] == 1:
            solar_index = 1
        else:
            solar_index = 0

        lv_index = 0
        if self.data[4] == 1:
            lv_index = 1
        else:
            lv_index = 0

        #Battery Indicator 
        pixmap_battery_indicator = QPixmap(self.battery_images[battery_index])
        self.battery_indicator_label.setPixmap(pixmap_battery_indicator.scaled(73, 40))

        #Motor Indicator 
        pixmap_motor_indicator = QPixmap(self.motor_images[motor_index])
        self.motor_indicator_label.setPixmap(pixmap_motor_indicator.scaled(73, 40))

        #Motor Controller Indicator 
        pixmap_motorcntrl_indicator = QPixmap(self.motorcntrl_images[motorcntrl_index])
        self.motorcntrl_indicator_label.setPixmap(pixmap_motorcntrl_indicator.scaled(73, 40))

        #MPPT Indicator 
        pixmap_mppt_indicator = QPixmap(self.mppt_images[mppt_index])
        self.mppt_indicator_label.setPixmap(pixmap_mppt_indicator.scaled(73, 40))

        #Solar Indicator 
        pixmap_solar_indicator = QPixmap(self.solar_images[solar_index])
        self.solar_indicator_label.setPixmap(pixmap_solar_indicator.scaled(73, 40))

        #LV Indicator 
        pixmap_lv_indicator = QPixmap(self.lv_images[lv_index])
        self.lv_indicator_label.setPixmap(pixmap_lv_indicator.scaled(73, 40))

        # Determine the index for the temperature image
        temp_index = 0
        if self.data[6] > 45 and self.data[6] < 60:
            temp_index = 1
        elif self.data[6] >= 60 and self.data[6] < 80:
            temp_index = 2

        #Cocpit Temperature
        pixmap_temp = QPixmap(self.temp_images[temp_index])
        painter_temp = QPainter(pixmap_temp)
        painter_temp.setFont(QFont('Good Times', 20))  
        painter_temp.setPen(Qt.white)  
        text_width_temp = painter_temp.fontMetrics().width(str(self.data[6]))
        x_temp = (pixmap_temp.width() - text_width_temp) // 2
        painter_temp.drawText(x_temp, 145, str(self.data[6]))
        painter_temp.end()
        self.temp1_label.setPixmap(pixmap_temp.scaled(71, 157))

        #Motor Temperature
        pixmap_temp2 = QPixmap(self.temp_images[temp_index])
        painter_temp2 = QPainter(pixmap_temp2)
        painter_temp2.setFont(QFont('Good Times', 20))
        painter_temp2.setPen(Qt.white)  
        text_width_temp2 = painter_temp2.fontMetrics().width(str(self.data[6]))
        x_temp = (pixmap_temp2.width() - text_width_temp2) // 2
        painter_temp2.drawText(x_temp, 145, str(self.data[6]))
        painter_temp2.end()
        self.temp2_label.setPixmap(pixmap_temp.scaled(71, 157))

        #Motor Controller Temperature
        pixmap_temp3 = QPixmap(self.temp_images[temp_index])
        painter_temp3 = QPainter(pixmap_temp3)
        painter_temp3.setFont(QFont('Good Times', 20)) 
        painter_temp3.setPen(Qt.white)  
        text_width_temp3 = painter_temp3.fontMetrics().width(str(self.data[6]))
        x_temp = (pixmap_temp3.width() - text_width_temp3) // 2
        painter_temp3.drawText(x_temp, 145, str(self.data[6]))
        painter_temp3.end()
        self.temp3_label.setPixmap(pixmap_temp.scaled(71, 157))

        #Battery Temperature
        pixmap_temp4 = QPixmap(self.temp_images[temp_index])
        painter_temp4 = QPainter(pixmap_temp4)
        painter_temp4.setFont(QFont('Good Times', 20)) 
        painter_temp4.setPen(Qt.white)  
        text_width_temp4 = painter_temp4.fontMetrics().width(str(self.data[6]))
        x_temp = (pixmap_temp4.width() - text_width_temp4) // 2
        painter_temp4.drawText(x_temp, 145, str(self.data[6]))
        painter_temp4.end()
        self.temp4_label.setPixmap(pixmap_temp.scaled(71, 157))

        #Current of motor 
        pixmap_motor = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 20))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[7])+ " AMP")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 43, str(self.data[7])+ " AMP")
        painter_motor.end()
        self.motor_current_fr.setPixmap(pixmap_motor.scaled(173, 63))

        #Voltage of motor 
        pixmap_motor = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_motor = QPainter(pixmap_motor)
        painter_motor.setFont(QFont('Good Times', 20))  
        painter_motor.setPen(Qt.white)  
        text_width_motor = painter_motor.fontMetrics().width(str(self.data[8])+ " AMP")
        x_temp = (pixmap_motor.width() - text_width_motor) // 2
        painter_motor.drawText(x_temp, 43, str(self.data[8])+ " AMP")
        painter_motor.end()
        self.motor_voltage_fr.setPixmap(pixmap_motor.scaled(173, 63))

        #Current of motor controller
        pixmap_motorctrl = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_motorctrl = QPainter(pixmap_motorctrl)
        painter_motorctrl.setFont(QFont('Good Times', 20))  # Set the font and size of the variable text
        painter_motorctrl.setPen(Qt.white)  # Set the color of the variable text
        text_width_motorctrl = painter_motorctrl.fontMetrics().width(str(self.data[9]) + " AMP")
        x_temp = (pixmap_motorctrl.width() - text_width_motorctrl) // 2
        painter_motorctrl.drawText(x_temp, 43, str(self.data[9]) + " AMP")
        painter_motorctrl.end()
        self.motorctrl_current_fr.setPixmap(pixmap_motorctrl.scaled(173, 63))

        #Voltage of motor controller
        pixmap_motorctrl = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_motorctrl = QPainter(pixmap_motorctrl)
        painter_motorctrl.setFont(QFont('Good Times', 20))  # Set the font and size of the variable text
        painter_motorctrl.setPen(Qt.white)  # Set the color of the variable text
        text_width_motorctrl = painter_motorctrl.fontMetrics().width(str(self.data[10]) + " AMP")
        x_temp = (pixmap_motorctrl.width() - text_width_motorctrl) // 2
        painter_motorctrl.drawText(x_temp, 43, str(self.data[10]) + " AMP")
        painter_motorctrl.end()
        self.motorctrl_voltage_fr.setPixmap(pixmap_motorctrl.scaled(173, 63))

        #Current of battery 
        pixmap_battery = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 20))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[11])+ " AMP")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 43, str(self.data[11])+ " AMP")
        painter_battery.end()
        self.battery_current_fr.setPixmap(pixmap_battery.scaled(173, 63))

        #Voltage of battery 
        pixmap_battery = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Drive time frame.png")
        painter_battery = QPainter(pixmap_battery)
        painter_battery.setFont(QFont('Good Times', 20))  # Set the font and size of the variable text
        painter_battery.setPen(Qt.white)  # Set the color of the variable text
        text_width_battery = painter_battery.fontMetrics().width(str(self.data[12])+ " AMP")
        x_temp = (pixmap_battery.width() - text_width_battery) // 2
        painter_battery.drawText(x_temp, 43, str(self.data[12])+ " AMP")
        painter_battery.end()
        self.battery_voltage_fr.setPixmap(pixmap_battery.scaled(173, 63))

        #Distance Travelled
        pixmap_distance = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Distance_time frame.png")
        painter_distance = QPainter(pixmap_distance)
        painter_distance.setFont(QFont('Good Times', 20))  # Set the font and size of the variable text
        painter_distance.setPen(Qt.white)  # Set the color of the variable text
        text_width_distance = painter_distance.fontMetrics().width(str(self.data[13])+"KM")
        x_temp = (pixmap_distance.width() - text_width_distance) // 2
        painter_distance.drawText(x_temp, 35, str(self.data[13])+" KM")
        painter_distance.end()
        self.distfr.setPixmap(pixmap_distance.scaled(175, 46))

        #Time
        # print(time_str, end='\r')
        pixmap_time = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Distance_time frame.png")
        painter_time = QPainter(pixmap_time)
        painter_time.setFont(QFont('Good Times', 15))  # Set the font and size of the variable text
        painter_time.setPen(Qt.white)  # Set the color of the variable text
        text_width_time = painter_time.fontMetrics().width(self.time_str +" HRs")
        x_temp = (pixmap_time.width() - text_width_time) // 2
        painter_time.drawText(x_temp, 33, self.time_str + " HRs")
        painter_time.end()
        self.timefr.setPixmap(pixmap_time.scaled(175, 46))

    def load_html_file(self, url):
            self.web_view = QWebEngineView(self)
            self.web_view.setGeometry(QRect(190, 32, 790, 444))
            self.web_view.setUrl(QUrl(url))
            self.web_view.hide()

    def ImageUpdateSlot(self, image):
        frame_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Rearviewframe.png")
        overlay_image = QImage(frame_image.size(), QImage.Format_ARGB32)
        # overlay_image.fill(Qt.transparent)

        painter = QPainter(overlay_image)
        painter.drawPixmap(0, 0, frame_image)
        painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
        painter.drawImage(5,5, image)
        painter.end()

        self.camera_label.setPixmap(QPixmap.fromImage(overlay_image))

    def Mainwindow(self):
        self.button1_clicked = True
        self.load_html_file("http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV2.html")
        # self.camera_label.setGeometry(QRect(180, 20, 806 , 510))
        self.hide()
        self.button1_clicked = False

    def Temps(self):
        self.button2_clicked = True
        self.hide()
        self.button2_clicked = False
        
    def Current(self):
        self.button3_clicked = True
        self.hide()
        self.button3_clicked = False
        
    def Map(self):
        self.button4_clicked = True
        self.hide()
        self.button4_clicked = False

    def CPanel(self):
        self.button5_clicked = True
        self.hide()
        self.button5_clicked = False

    def Fans(self):
        self.button6_clicked = True
        self.hide()
        self.button6_clicked = False

    def Lights(self):
        self.button7_clicked = True
        self.hide()
        self.button7_clicked = False

    def State(self):
        self.button8_clicked = True
        self.hide()
        self.button8_clicked = False

    def TBD(self):
        self.button9_clicked = True
        self.hide()
        self.button9_clicked = False

    def manual_button1_click(self):
        if self.button10.isChecked():
            self.slider1.show()
            self.slider1_label.show()

        else:
            self.slider1.hide()
            self.slider1_label.hide()

    def manual_button2_click(self):
        if self.button11.isChecked():
            self.slider2.show()
            self.slider2_label.show()

        else:
            self.slider2.hide()
            self.slider2_label.hide()

    def manual_button3_click(self):
        if self.button12.isChecked():
            self.slider3.show()
            self.slider3_label.show()

        else:
            self.slider3.hide()
            self.slider3_label.hide()

    def update_slider1(self, value):
        self.slider1_label.setText(str(value))

    def update_slider2(self, value):
        self.slider2_label.setText(str(value))

    def update_slider3(self, value):
        self.slider3_label.setText(str(value))
        
    def light_button_click(self):
        pass

    def seq_button_click(self):
        pass

    def state_button_click(self):
        pass

            
class Camera(QThread):
    ImageUpdate = pyqtSignal(QImage)

    def run(self):
        self.ThreadActive = True
        capture = cv2.VideoCapture(0)
        while self.ThreadActive:
            ret, frame = capture.read()
            if ret:
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                flipped_image = cv2.flip(image, 1)
                convert_to_qt_format = QImage(flipped_image.data, flipped_image.shape[1], flipped_image.shape[0],
                                              QImage.Format_RGB888)
                pic = convert_to_qt_format.scaled(797, 500)
                self.ImageUpdate.emit(pic)

    def stop(self):
        self.ThreadActive = False
        self.quit()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
