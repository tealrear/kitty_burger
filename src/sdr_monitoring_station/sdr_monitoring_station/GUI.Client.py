import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer,Qt
from PySide6.QtGui import QPixmap

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
# from .consol_ui import Ui_Form
from.gui_ui import Ui_Form
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import BatteryState

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


class Tsar_Node(Node):
   def __init__(self):
    super().__init__('gui_controller')

    self.declare_parameter('qos_depth', 10)
    qos_depth = self.get_parameter('qos_depth').value #현재값을 가져온다
    self.callback_group = ReentrantCallbackGroup()

    qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,#꼭 전달해라
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)#과거는 잊고 현재 것만 준다

    self.pub = self.create_publisher(
            String,
            'ui_pub_sub',
            qos,
            )


#라즈베리 제어
    self.face_pub = self.create_publisher(String, '/face_cmd',qos)
    self.buzzer_pub = self.create_publisher(String, '/buzzer_cmd', qos)
    self.tail_pub = self.create_publisher(String, '/tail_cmd', qos)
#라즈베리 제어

    self.battery=None #잔량표시를 위한 변수

    self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',#ros2 topic list로 노드 확인
            self.cb_battery,
            qos,
            )

   def cb_battery(self,msg:BatteryState):
       self.battery = msg
       self.get_logger().info(
        f"Battery recv: voltage={msg.voltage}, percentage={msg.percentage}"
    )



    # self.stop_service_clint = self.create_client(String, 'stop_service')
    # self.stop_clint = stop_service_clint.Request()

class MainWindow(QMainWindow):

    def __init__(self, tsar_node: Tsar_Node): #type이 함수니까
        super().__init__()
        self.tsar = tsar_node


        self.ui = Ui_Form()
        self.ui.setupUi(self)

        # button clicked 이벤트 핸들러로 button_clicked 함수와 연결한다.
        self.ui.btn_go.clicked.connect(self.btn_go_Function)
        self.ui.btn_back.clicked.connect(self.btn_back_Function)
        self.ui.btn_right.clicked.connect(self.btn_right_Function)
        self.ui.btn_left.clicked.connect(self.btn_left_Function)
        self.ui.btn_stop.clicked.connect(self.btn_stop_Function)
    #-------------------------------------
        self.ui.btn_face.clicked.connect(self.publish_face)
    #-------------------------------------


        #페이지 이동
        self.ui.btn_next_0.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.btn_pre_0.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.btn_next_1.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(2))
        self.ui.btn_pre_1.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.btn_next_2.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.btn_pre_2.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.btn_next_3.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.btn_pre_3.clicked.connect(lambda : self.ui.stackedWidget.setCurrentIndex(2))

        self.linear = 0.0
        self.angular = 0.0
        self.step = 0.01

        #배터리 표시
        self.ui.battery_label.setText("Battery: no data")
        self.battery_timer = QTimer(self)
        self.battery_timer.timeout.connect(self.update_battery_label)
        self.battery_timer.start(200)

    def publish_ui(self):
        msg = String()
        msg.data = f"{self.linear} {self.angular}"   # 반드시 문자열!
        self.tsar.pub.publish(msg)
        self.tsar.get_logger().info(f"Published ui_pub_sub: {msg.data}")
        self.ui.listWidget.addItem(f"linear:{self.linear},angular{self.angular}")

#rasp ----------------------------------------------
    def publish_face(self):
        text = self.ui.Face_lineEdit.text().strip().lower()
        if not text:
            return

        face_msg = String()
        tail_msg = String()
        buzzer_msg = String()

        if text == "angry":
            face_msg.data = "angry"
            tail_msg.data = "angry"
            buzzer_msg.data = "warning"
        elif text == "heart":
            face_msg.data = "heart"
            tail_msg.data = "friendly"
            buzzer_msg.data = "happy"
        elif text == "neutral":
            face_msg.data = "neutral"
            tail_msg.data = "normal"
            buzzer_msg.data = "stop"
        elif text == "cry":
            face_msg.data = "cry"
            tail_msg.data = "stop"
            buzzer_msg.data = "danger"
        else:
            face_msg.data = text
            tail_msg.data = "stop"
            buzzer_msg.data = "stop"

        self.tsar.face_pub.publish(face_msg)
        self.tsar.tail_pub.publish(tail_msg)
        self.tsar.buzzer_pub.publish(buzzer_msg)

        self.face_list(face_msg.data)


    def face_list(self,text:str):
        self.ui.Face_listWidget.addItem(text)

        image_map = {
            "angry": os.path.join(BASE_DIR, "face", "angry.jpg"),
            "neutral": os.path.join(BASE_DIR, "face", "neutral.jpg"),
            "cry": os.path.join(BASE_DIR, "face", "cry.jpg"),
            "heart": os.path.join(BASE_DIR, "face", "heart.jpg"),
            "message": os.path.join(BASE_DIR, "face", "message.jpg"),
            "blink": os.path.join(BASE_DIR, "face", "blink.jpg"),
        }

        path = image_map.get(text) #파일 경로 문자열이 들어가.
        #"angry"라는 글자를 다시 주는 게 아니라,그 key에 대응하는 value, 즉 이미지 경로를 줘.

        if not path:
            self.ui.face_label.setText(f"이미지 없음\n{text}")
            return

        pixmap = QPixmap(path)#QPixmap은 이미지 파일을 읽어서 Qt에서 화면에 띄울 수 있는 그림 객체로 만드는 클래스야.

        if pixmap.isNull():
            self.ui.face_label.setText(f"파일 없음\n{text}")
            return

        self.ui.face_label.setPixmap(
            pixmap.scaled( # 이미지를 라벨 크기에 맞게 조절
                self.ui.face_label.size(), # face_label의 현재 크기에 맞춤
                Qt.KeepAspectRatio,# 이미지 비율 유지
                Qt.SmoothTransformation # 부드럽게 크기 변환
        )
    )
#rasp ----------------------------------------------

    def btn_go_Function(self):
        self.linear += self.step
        self.angular = 0
        self.publish_ui()

    def btn_back_Function(self):
        self.linear -= self.step
        self.angular = 0
        self.publish_ui()

    def btn_left_Function(self):
        self.angular += self.step
        self.publish_ui()

    def btn_right_Function(self):
        self.angular -= self.step
        self.publish_ui()

    def btn_stop_Function(self):
        self.linear =0
        self.angular = 0
        self.publish_ui()

#-------------------------------------
   #배터리 부분
    def update_battery_label(self):
        if self.tsar.battery is None:
            self.ui.battery_label.setText("Battery: no data")
            return

        battery = self.tsar.battery

    # percentage가 0.0~1.0 형태로 오는 경우가 많음
        if battery.percentage >= 0.0:
            percent = battery.percentage
            self.ui.battery_label.setText(
            f"Battery: {percent:.2f}% ({battery.voltage:.2f}V)"
        )
        else:
            self.ui.battery_label.setText(
            f"Battery: {battery.voltage:.2f}V"
        )
#---------------------------------------------------


def main(args=None):
  rclpy.init(args=args)#ROS2 파이썬을 초기화.
  app = QApplication(sys.argv)#Qt 앱(이벤트 루프) 객체 생성.
  node = Tsar_Node()
  window = MainWindow(node)
  window.show()

  spin_timer = QTimer()
  spin_timer.timeout.connect(lambda: rclpy.spin_once(node,timeout_sec=0.0))#QTimer가 시간이 될 때마다 발생
  spin_timer.start(10)#10ms마다 timeout 신호 발생
  try:
    end=app.exec()   # Qt 이벤트 루프 실행 (여기가 메인)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
    end =0
  finally:
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(end)


if __name__ == '__main__':
  main()
