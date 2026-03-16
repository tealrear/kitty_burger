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
        self.current_state = "ACT0_SLEEPY" # 현재 상태 저장 변수

        # 1. 미션 상태 구독 추가
        self.state_sub = self.create_subscription(String, '/mission_state', self.cb_state, 10)

        # 2. 모든 영상 토픽 구독 (이름만 다르게)
        # 메인 뷰 (C++에서 쏘는 것)
        self.create_subscription(CompressedImage, '/vision/processed/compressed', 
                                 lambda msg: self.image_router(msg, "MAIN"), 10)
        # AI 뷰 (얼굴/손 ROI - detect_human에서 쏘는 것)
        self.create_subscription(CompressedImage, '/vision/roi/compressed', 
                                 lambda msg: self.image_router(msg, "AI"), 10)
        # 숫자 뷰 (숫자 박스 - digit_reader에서 쏘는 것)
        self.create_subscription(CompressedImage, '/vision/digit_debug/compressed', 
                                 lambda msg: self.image_router(msg, "DIGIT"), 10)

    def cb_state(self, msg): self.current_state = msg.data

    # 상태에 따라 latest_frame에 저장될 이미지를 선택
    def image_router(self, msg, source):
        """현재 상태에 맞는 영상원만 latest_frame에 업데이트함"""
        # 상태별 우선순위 로직
        should_update = False
        
        if self.current_state == "ACT3_AUTHENTICATE" and source == "AI":
            should_update = True
        elif self.current_state == "ACT4_DELIVERY" and source == "DIGIT":
            should_update = True
        # [수정] AI가 작동 중이더라도 MAIN 영상은 계속 업데이트되게 하여 '멈춤' 느낌 삭제
        elif source == "MAIN":
            should_update = True

        if should_update:
            try:
                arr = np.frombuffer(msg.data, np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    self.latest_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            except Exception as e:
                self.get_logger().error(f"Router Error: {e}")

    def cb_battery(self, msg: BatteryState):
        self.battery = msg

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
        """메인 카메라 영상을 camera_label에 업데이트합니다."""
        if self.tsar.latest_frame is not None:
            self.set_image_to_label(self.tsar.latest_frame, self.ui.camera_label)
    
    def set_image_to_label(self, frame, label):
        """이미지 데이터를 Qt Pixmap으로 변환하여 라벨에 출력합니다."""
        h, w, ch = frame.shape
        qimg = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)
        # 라벨 크기에 맞춰 비율 유지하며 출력
        label.setPixmap(pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
    
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
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0) if rclpy.ok() else None)
    ros_timer.start(10)

    try:
        app.exec() # sys.exit()을 빼고 try-finally 구조로 변경
    except KeyboardInterrupt:
        pass
    finally:
        # [핵심] 종료 시 정지 명령 전송 (순서 중요)
        if rclpy.ok():
            try:
                stop_t = Twist()
                node.pub.publish(stop_t) # 정지 명령
            except:
                pass
        
        # 노드 파괴 후 셧다운
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()