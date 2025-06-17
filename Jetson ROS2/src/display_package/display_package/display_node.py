
import sys
import cv2
import time
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QSlider, QMessageBox, QDial
from PyQt5.QtGui import QPixmap, QIcon, QImage, QPalette, QBrush, QPainter, QFont, QColor
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QRect, QUrl, QTimer, QSize
import random
from array import array
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as rosarray
import pandas as pd

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
        #self.setWindowFlag(Qt.FramelessWindowHint)
        self.topic_name = "final_data"
        self.connect_ros()

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        #self.setGeometry(0,0,820,480)
        self.setFixedSize(800,480)

        pixmap = QPixmap("/home/agnirath/Agnirath25/src/display_package/Dashboard/assets/Bg.png")
        self.central_widget.setAutoFillBackground(True)
        palette = self.central_widget.palette()
        palette.setBrush(QPalette.Window, QBrush(pixmap))
        self.central_widget.setPalette(palette)

        self.camera_label = QLabel(self)
        self.camera_label.setGeometry(QRect(0, 0, 800, 440))
        self.cam = Camera()

        self.camera_label.show()
        self.cam.ImageUpdate1.connect(self.ImageUpdateSlot)
        self.cam.start()

        self.popup_active = False

        '''self.msgBox = QMessageBox()
        self.msgBox.setText("mei kya hi bolu")
        self.msgBox.show()
        time.sleep(10)
        self.msgBox.show()'''

        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.update_variable)
        self.timer1.start(5000) 

    def connect_ros(self):
        try:
            # ROS2 init
            rclpy.init(args=None)
            self.node = Node("display_node")
            self.sub = self.node.create_subscription(
                rosarray,
                self.topic_name,
                self.sub_rosarray_callback,
                10,
            )        
            # Create a publisher
            ''' self.pub = self.node.create_publisher(
                rosarray,
                'control_data',
                10
            )'''
            
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

    '''def publish_data(self, data):
        msg = rosarray()
        msg.data = data
        self.pub.publish(msg)'''

    # def publish_random_data(self):
    #     # Generate random data
    #     data = array('f', [random.random()])
    #     print(data)
    #     self.publish_data(data)
        
        
    ### ROS2 Data Updater
    def sub_rosarray_callback(self, msg):
        self.data = msg.data
        #print(self.data)

    def show_popup(self, codes):
        if self.popup_active:
            return  # Skip if already showing popups

        self.popup_active = True

        error_data = pd.read_csv('/home/agnirath/Agnirath25/src/display_package/Dashboard/new_error_code.csv')
        self.error_mapping = {
            int(row['code']): {'error': row['error'], 'action': row['action']}
            for _, row in error_data.iterrows()
        }

        valid_codes = [int(c) for c in codes if int(c) in self.error_mapping and int(c) != 0]

        if not valid_codes:
            self.popup_active = False
            return

        def show_message(index):
            if index >= len(valid_codes):
                self.popup_active = False
                return

            error_code = valid_codes[index]
            error_info = self.error_mapping[error_code]
            msg = QMessageBox()
            msg.setWindowTitle("Error Flags")
            msg.setText(error_info['error'])
            msg.setIcon(QMessageBox.Warning)
            msg.setInformativeText(error_info['action'])
            msg.move(600, 0)
            msg.show()

            QTimer.singleShot(5000, lambda: (msg.close(), show_message(index + 1)))

        show_message(0)



    def update_variable(self):
        print(f"Data slice for popup: {self.data[15:23]}")
        self.show_popup(self.data[15:23])

    def ImageUpdateSlot(self, image):
        frame_image = QPixmap("/home/agnirath/Agnirath25/src/display_package/Dashboard/assets/rearviewframe3.png")
        overlay_image = QImage(frame_image.size(), QImage.Format_ARGB32)
        # overlay_image.fill(Qt.transparent)

        painter = QPainter(overlay_image)
        painter.drawPixmap(0, 0, frame_image)
        painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
        painter.drawImage(5, 5, image)
        painter.end()

        self.camera_label.setPixmap(QPixmap.fromImage(overlay_image))

class Camera(QThread):
    def __init__(self):
        super().__init__()
        self.k = True
    
    ImageUpdate1 = pyqtSignal(QImage)
    
    def run(self):
        self.ThreadActive = True
        self.capture = cv2.VideoCapture(0)
        while self.ThreadActive:
            
            if not self.capture.isOpened():
                self.try_reconnect()
                continue
            
            ret, frame = self.capture.read()
            if ret:
                image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                flipped_image = cv2.flip(image, 1)
                convert_to_qt_format = QImage(image.data, image.shape[1], image.shape[0],
                                              QImage.Format_RGB888)
                
                if (self.k==True):
                    pic = convert_to_qt_format.scaled(790, 432)
                else:
                    pic = convert_to_qt_format.scaled(264, 165)
                    
                self.ImageUpdate1.emit(pic)
                
            else:
                
                self.capture.release()
                self.try_reconnect()


    def stop(self):
        self.ThreadActive = False
        if self.capture.isOpened():
            self.capture.release()
        self.quit()
        #self.wait()
        #if self.capture is not None:
            #self.capture.release()
            
    def try_reconnect(self):
        
        while self.ThreadActive:
            self.capture = cv2.VideoCapture(0)
            if self.capture.isOpened():
                print("Reconnected to Webcam")
                break
            QThread.msleep(1000)

def main():
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
