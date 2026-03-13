import sys
import os
import rclpy
import cv2
import numpy as np
import json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QPixmap, QImage

# UI 파일 로드
from .gui_ui import Ui_Form

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class Tsar_Node(Node):
    def __init__(self):
        super().__init__('gui_controller')

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 퍼블리셔 설정
        self.pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.face_pub = self.create_publisher(String, '/face_cmd', qos)
        self.buzzer_pub = self.create_publisher(String, '/buzzer_cmd', qos)
        self.tail_pub = self.create_publisher(String, '/tail_cmd', qos)

        # 서브스크라이버 설정
        self.battery = None
        self.battery_sub = self.create_subscription(BatteryState, '/battery_state', self.cb_battery, qos)
        
        # 영상 스트리밍 구독 추가
        self.latest_frame = None
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/vision/processed/compressed',
            self.cb_image,
            10
        )

    def cb_battery(self, msg: BatteryState):
        self.battery = msg

    def cb_image(self, msg: CompressedImage):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is not None:
                # BGR -> RGB 변환하여 저장
                self.latest_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Image Decode Error: {e}")

class MainWindow(QMainWindow):
    def __init__(self, tsar_node: Tsar_Node):
        super().__init__()
        self.tsar = tsar_node
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.setWindowTitle("부기 통합 관제 모니터")

        # --- 버튼 이벤트 연결 ---
        self.ui.btn_go.clicked.connect(self.btn_go_Function)
        self.ui.btn_back.clicked.connect(self.btn_back_Function)
        self.ui.btn_right.clicked.connect(self.btn_right_Function)
        self.ui.btn_left.clicked.connect(self.btn_left_Function)
        self.ui.btn_stop.clicked.connect(self.btn_stop_Function)
        self.ui.btn_face.clicked.connect(self.publish_face)

        # --- 페이지 이동 버튼 ---
        self.ui.btn_next_0.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.btn_pre_0.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.btn_next_1.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))
        self.ui.btn_pre_1.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.btn_next_2.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.btn_pre_2.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.btn_next_3.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.btn_pre_3.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))

        # 제어 변수
        self.linear = 0.0
        self.angular = 0.0
        self.step = 0.05 # 이동 속도 증분

        # --- 타이머 설정 ---
        # 1. 배터리 업데이트 타이머
        self.battery_timer = QTimer(self)
        self.battery_timer.timeout.connect(self.update_battery_label)
        self.battery_timer.start(500)

        # 2. 영상 업데이트 타이머 (30 FPS)
        self.video_timer = QTimer(self)
        self.video_timer.timeout.connect(self.update_video_stream)
        self.video_timer.start(33)

    def update_video_stream(self):
        """현재 활성화된 페이지에 따라 영상을 출력합니다."""
        if self.tsar.latest_frame is None:
            return

        frame = self.tsar.latest_frame
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        curr_idx = self.ui.stackedWidget.currentIndex()

        if curr_idx == 0:  # 0페이지: face_label
            scaled_pixmap = pixmap.scaled(self.ui.face_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.ui.face_label.setPixmap(scaled_pixmap)
        elif curr_idx == 1:  # 1페이지: camera_label
            scaled_pixmap = pixmap.scaled(self.ui.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.ui.camera_label.setPixmap(scaled_pixmap)

    def update_battery_label(self):
        if self.tsar.battery is None:
            self.ui.battery_label.setText("Battery: no data")
            return
        
        b = self.tsar.battery
        # percentage가 0.0~1.0 이면 100을 곱함
        percent = b.percentage if b.percentage > 1.0 else b.percentage * 100
        self.ui.battery_label.setText(f"Battery: {percent:.1f}% ({b.voltage:.1f}V)")

    def publish_ui(self):
        msg = Twist()
        msg.linear.x = float(self.linear)
        msg.angular.z = float(self.angular)
        self.tsar.pub.publish(msg)
        self.ui.listWidget.addItem(f"CMD: L={self.linear:.2f}, A={self.angular:.2f}")
        self.ui.listWidget.scrollToBottom()

    def publish_face(self):
        text = self.ui.Face_lineEdit.text().strip().lower()
        if not text: return

        face_msg = String(); tail_msg = String(); buzzer_msg = String()
        
        # 감정표현 매핑 (기존 로직 유지)
        mapping = {
            "angry": ("angry", "angry", "warning"),
            "heart": ("heart", "friendly", "happy"),
            "neutral": ("neutral", "normal", "stop"),
            "cry": ("cry", "stop", "danger")
        }

        f, t, b = mapping.get(text, (text, "stop", "stop"))
        face_msg.data = f; tail_msg.data = t; buzzer_msg.data = b

        self.tsar.face_pub.publish(face_msg)
        self.tsar.tail_pub.publish(tail_msg)
        self.tsar.buzzer_pub.publish(buzzer_msg)
        
        self.ui.Face_listWidget.addItem(f"Face Set: {text}")
        self.show_static_face(text)

    def show_static_face(self, text):
        """명령을 내렸을 때 로컬 이미지를 face_label에 잠시 띄웁니다."""
        path = os.path.join(BASE_DIR, "face", f"{text}.jpg")
        if os.path.exists(path):
            pixmap = QPixmap(path)
            self.ui.face_label.setPixmap(pixmap.scaled(self.ui.face_label.size(), Qt.KeepAspectRatio))

    # --- 제어 함수들 ---
    def btn_go_Function(self): self.linear += self.step; self.angular = 0.0; self.publish_ui()
    def btn_back_Function(self): self.linear -= self.step; self.angular = 0.0; self.publish_ui()
    def btn_left_Function(self): self.angular += 0.1; self.publish_ui()
    def btn_right_Function(self): self.angular -= 0.1; self.publish_ui()
    def btn_stop_Function(self): self.linear = 0.0; self.angular = 0.0; self.publish_ui()

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = Tsar_Node()
    window = MainWindow(node)
    window.show()

    # ROS 2 스핀을 위한 타이머
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    try:
        sys.exit(app.exec())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()