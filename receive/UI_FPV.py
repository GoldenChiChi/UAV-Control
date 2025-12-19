# -*- coding: latin-1 -*-
import sys
import cv2
import json
import socket
from PyQt5 import QtWidgets, QtGui, QtCore

class VideoThread(QtCore.QThread):
    change_pixmap_signal = QtCore.pyqtSignal(QtGui.QImage)
    no_signal = QtCore.pyqtSignal()

    def run(self):
        while True:
            cap = cv2.VideoCapture(20)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            if not cap.isOpened():
                self.no_signal.emit()
                QtCore.QThread.msleep(500)
                continue

            while cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    h, w, ch = rgb_image.shape
                    bytes_per_line = ch * w
                    qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
                    self.change_pixmap_signal.emit(qt_image)
                else:
                    self.no_signal.emit()
                    break

            cap.release()
            QtCore.QThread.msleep(500)

class UDPThread(QtCore.QThread):
    data_signal = QtCore.pyqtSignal(str)
    no_data_signal = QtCore.pyqtSignal()

    def run(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(('0.0.0.0', 12345))

        while True:
            try:
                udp_socket.settimeout(1.0)
                data, _ = udp_socket.recvfrom(1024)
                json_data = json.loads(data.decode('utf-8'))
                self.data_signal.emit(json.dumps(json_data, indent=4))
            except socket.timeout:
                self.no_data_signal.emit()
            except json.JSONDecodeError:
                continue

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Camera and UDP JSON Viewer")
        self.setGeometry(100, 100, 1200, 500)

        self.video_label = QtWidgets.QLabel(self)
        self.video_label.setText("wait video signal ...")

        self.json_text = QtWidgets.QTextEdit(self)
        self.json_text.setReadOnly(True)
        self.json_text.setPlainText("wait data signal ...")

        # 创建一个标签用于居中显示文字
        status_label = QtWidgets.QLabel("GKRY(ZDLD) FPV CTRL ", self)
        status_label.setAlignment(QtCore.Qt.AlignCenter)

        # 设置水平布局
        h_layout = QtWidgets.QHBoxLayout()
        h_layout.addWidget(self.video_label)
        h_layout.addWidget(self.json_text)

        # 设置垂直布局
        v_layout = QtWidgets.QVBoxLayout(self)
        v_layout.addLayout(h_layout)
        v_layout.addWidget(status_label)

        self.setLayout(v_layout)

        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.no_signal.connect(self.no_video_signal)
        self.video_thread.start()

        self.udp_thread = UDPThread()
        self.udp_thread.data_signal.connect(self.update_json)
        self.udp_thread.no_data_signal.connect(self.no_json_data)
        self.udp_thread.start()

    def update_image(self, qt_image):
        self.video_label.setPixmap(QtGui.QPixmap.fromImage(qt_image))

    def no_video_signal(self):
        self.video_label.setText("wait video signal ...")

    def update_json(self, json_str):
        self.json_text.setPlainText(json_str)

    def no_json_data(self):
        self.json_text.setPlainText("wait data signal ...")

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
