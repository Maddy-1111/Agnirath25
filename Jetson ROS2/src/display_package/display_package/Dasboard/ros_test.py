import sys
import cv2
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel
from PyQt5.QtGui import QPixmap, QImage, QPalette, QBrush, QPainter
from PyQt5.QtCore import QThread, pyqtSignal, QRect, QTimer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
from array import array
import random


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

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.setFixedSize(1024, 600)

        pixmap = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Bg.png")
        self.central_widget.setAutoFillBackground(True)
        palette = self.central_widget.palette()
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.central_widget.setPalette(palette)

        self.camera_label = QLabel(self)
        self.camera_label.setGeometry(QRect(185, 340, 631 , 230))
        self.cam = Camera()

        self.camera_label.show()
        self.cam.ImageUpdate.connect(self.ImageUpdateSlot)
        self.cam.start()

        # Create a QTimer
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_random_data)
        self.timer.start(1000)

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
            self.ros_thread = RosThread(self.node)   # Create ros thread 
            self.ros_thread.start() 
        
            # spin once, timeout_sec 5[s]
            timeout_sec_rclpy = 5
            timeout_init = time.time()
            rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
            timeout_end = time.time()
            ros_connect_time = timeout_end - timeout_init

            # Error Handle for rclpy timeout
            if ros_connect_time >= timeout_sec_rclpy:
                print("Couldn't Connect")
                
            else:
                print("Connected")
  

              
        except:
            pass

    def publish_data(self, data):
        msg = rosarray()
        msg.data = data
        self.pub.publish(msg)

    def publish_random_data(self):
        # Generate random data
        data = array('f', [random.random()])
        self.publish_data(data)

    ### ROS2 Data Updater
    def sub_rosarray_callback(self, msg):
        self.data = msg.data
        print(self.data)

    def ImageUpdateSlot(self, image):
        frame_image = QPixmap("/home/veadesh/Agnirath/GUI/Beaglebone/Dashboard/assets/Rearviewframe.png")
        overlay_image = QImage(frame_image.size(), QImage.Format_ARGB32)
        # overlay_image.fill(Qt.transparent)

        painter = QPainter(overlay_image)
        painter.drawPixmap(0, 0, frame_image)
        painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
        painter.drawImage(5, 5, image)
        painter.end()

        self.camera_label.setPixmap(QPixmap.fromImage(overlay_image))

    def Mainwindow(self):
        self.button1_clicked = True
        self.load_html_file("http://127.0.0.1:5500/GUI/Beaglebone/Dashboard/mapV2.html")
        self.camera_label.setGeometry(QRect(185, 340, 631 , 230))
        self.hide()
        self.button1_clicked = False

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
                pic = convert_to_qt_format.scaled(620, 215)
                self.ImageUpdate.emit(pic)

    def stop(self):
        self.ThreadActive = False
        self.quit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())