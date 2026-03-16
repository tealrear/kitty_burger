import rclpy, time, json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan # 라이다 사용을 위한 임포트
from rclpy.qos import qos_profile_sensor_data # 상단에 추가

# 
class SdrMissionController(Node):
    def __init__(self):
        super().__init__('sdr_mission_controller')

        # QoS 설정 (rasp_face.py와 일치시킴)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 0. 설정값
        self.LIDAR_SAFE_DISTANCE = 0.5  # 50cm 이내에 물체가 있으면 장애물로 간주
        
        # 상태 정의
        # ACT0: 졸음주행 / ACT1: 장애물감지후퇴 / ACT2: 장애물제거대기 / ACT2_BYPASS: 우회
        # ACT4: 숫자 인식 및 이동(장애물 제거후 실행) / ACT5: 결제
        self.state = "ACT0_SLEEPY"
        self.munchi_count = 0
        self.wait_start_time = 0
        self.move_duration = 0 # 임시 주행 시간 설정용
        
        # 데이터 저장 변수
        self.last_obj = "NONE" 
        self.lidar_obstacle = False
        self.current_face = "none"
        self.current_gesture = "none"
        self.current_digit = "none"

        self.current_expression = "none"

        self.last_sent_face = ""
        self.last_sent_buzzer = ""
        self.last_sent_tail = ""

        self.gesture_count = 0

        # 결제 판정용 변수 추가
        self.payment_color_start_time = 0.0
        self.current_observed_color = "NONE"
        self.CONFIRM_DURATION = 2.0  # 판정에 필요한 시간 (초). 5초는 너무 길 수 있어 2초로 제안합니다.

        # --- 최적화용 딱 두 줄만 추가 ---
        self.last_state = None          # 이전 상태를 기억해서 '상태 변화'를 감지함
        self.state_start_time = time.time() # 상태가 바뀐 시점을 기록함

        # --- [모듈화용 저장소] ---
        self.confirm_timers = {} # 각 카테고리별(digit, color 등) 타이머 관리용 딕셔너리
        self.last_state = None
        self.state_start_time = time.time()

        # 구독 설정
        self.create_subscription(String, '/vision_fast_data', self.vision_cb, 10)
        self.create_subscription(String, '/person/hand', self.hand_cb, 10)
        self.create_subscription(String, '/person/digit', self.digit_cb, 10) 
        self.create_subscription(String, '/person/face_id', self.face_cb, 10)
        self.create_subscription(String, '/person/expression', self.exp_cb, 10)

        self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data) # 라이다 토픽 구독
        
        # --- [수정] rasp_face.py에 맞춘 발행 설정 ---
        self.face_pub = self.create_publisher(String, '/face_cmd', qos_profile)
        self.buzzer_cmd_pub = self.create_publisher(String, '/buzzer_cmd', qos_profile) # buzzer_pub에서 이름 변경
        self.tail_pub = self.create_publisher(String, '/tail_cmd', qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # 다른 노드들에게 현재 상태를 알림 (숫자 인식 제한용)
        self.state_pub = self.create_publisher(String, '/mission_state', qos_profile)

        self.create_timer(0.1, self.main_loop)
        self.get_logger().info("🚀 [SDR] 장애물 제거 후 숫자 인식 시나리오 가동")

    # --- [모듈 1: 시나리오 전환 후 안정화 체크] ---
    def is_scenario_stable(self, now, delay=2.0):
        """상태가 바뀐 후 delay만큼 시간이 지났는지 확인"""
        return (now - self.state_start_time) > delay

    # --- [모듈 2: 특정 값이 X초 동안 유지되는지 체크] ---
    def is_value_confirmed(self, category, new_val, now, threshold=2.0):
        """
        category: "digit", "color", "gesture" 등
        new_val: 현재 들어오는 센서/인식 값
        threshold: 유지해야 하는 시간(초)
        """
        # 해당 카테고리가 처음 들어오면 초기화
        if category not in self.confirm_timers:
            self.confirm_timers[category] = {"val": None, "start_time": now}

        timer_data = self.confirm_timers[category]

        # 인식된 값이 없거나 "none"이면 타이머 초기화 및 False 반환
        if new_val in ["none", "NONE", None]:
            timer_data["val"] = new_val
            timer_data["start_time"] = now
            return False

        # 값이 바뀌면 타이머 리셋
        if new_val != timer_data["val"]:
            timer_data["val"] = new_val
            timer_data["start_time"] = now
            return False

        # 값이 똑같이 유지되고 있다면 시간 계산
        duration = now - timer_data["start_time"]
        return duration >= threshold

    def face_cb(self, msg): 
        self.current_face = msg.data
        print("self.current_face : ", self.current_face)

    def vision_cb(self, msg): 
        self.last_obj = msg.data.split(':')[0]
        print("vision_sb msg : ", msg)

    def digit_cb(self, msg): self.current_digit = msg.data

    def hand_cb(self, msg):
        try: self.current_gesture = json.loads(msg.data).get("gesture", "none")
        except: pass
    
    def exp_cb(self, msg):
        try: self.current_expression = json.loads(msg.data).get("expression", "none")
        except: pass

    # 라이다 콜백: 전방 30도 범위 내의 최소 거리를 체크
    def lidar_cb(self, msg):
        front_ranges = msg.ranges[0:15] + msg.ranges[-15:] 
        valid_ranges = [r for r in front_ranges if r > 0.05]
        if valid_ranges:
            min_dist = min(valid_ranges)
            self.lidar_obstacle = (min_dist < self.LIDAR_SAFE_DISTANCE)
            print("min_dist face : ", min_dist)
            print("self.lidar_obstacle face : ", self.lidar_obstacle)
        print("front_ranges face : ", front_ranges)
        print("valid_ranges face : ", valid_ranges)

    # --- 하드웨어 통합 명령 전송 함수 ---
    def send_robot_cmd(self, face=None, buzzer=None, tail=None):
        """값이 바뀔 때만 전송하여 I2C 통신 마비를 방지합니다."""
        if face and face != self.last_sent_face:
            self.face_pub.publish(String(data=face))
            self.last_sent_face = face
            print("face face : ", face)

        if buzzer and buzzer != self.last_sent_buzzer:
            self.buzzer_cmd_pub.publish(String(data=buzzer))
            self.last_sent_buzzer = buzzer
            print("buzzer buzzer", buzzer)

        if tail and tail != self.last_sent_tail:
            self.tail_pub.publish(String(data=tail))
            self.last_sent_tail = tail
            print("tail tail : ", tail)


    def main_loop(self):
        t = Twist()
        now = time.time()

        # [추가] 1. 상태가 바뀌었는지 감지 (바뀌는 순간 타이머 리셋)
        if self.state != self.last_state:
            self.get_logger().info(f"🔄 상태 변경 감지: {self.last_state} -> {self.state},  last_obj is {self.last_obj}")
            self.state_start_time = now
            self.last_obj = "NONE" # 이전 단계의 색상 기억 삭제
            self.last_state = self.state
            self.confirm_timers.clear() # 상태가 바뀌면 모든 인식 대기열 초기화

        # [추가] 2. 상태 전환 후 '안정화 시간' 계산 (1초)
        is_stable = (now - self.state_start_time) > 0.5  # 1초 지나면 True

        # 장애물 여부 종합 (비전에서 파란색 감지 OR 라이다에서 근접 물체 감지)
        has_obstacle = (self.last_obj == "BLUE" or self.lidar_obstacle)
        print(f"Lidar: {self.lidar_obstacle}, Vision: {self.last_obj}, Combined: {has_obstacle}")

        # 현재 상태 브로드캐스팅
        self.state_pub.publish(String(data=self.state))


        # 1단계: 졸음 주행
        if self.state == "ACT0_SLEEPY":
            self.get_logger().info("꾸벅꾸벅 조는 표정")
            self.send_robot_cmd(face="sleepy", tail="stop") # 꾸벅꾸벅 조는 표정
            t.linear.x = 0.01
            t.angular.z = 0.1 if int(now * 2) % 2 == 0 else -0.1

            # 장애물이 있으면
            if has_obstacle:
                self.get_logger().info("⚠️ 장애물 감지! 후퇴 모드 진입")
                self.state = "ACT1_ALARM"
                self.munchi_count = 0

        # 2단계: 장애물 발견 및 3단 후퇴
        elif self.state == "ACT1_ALARM":
            self.get_logger().info("놀란 표정")
            self.send_robot_cmd(face="surprise", buzzer="warning")
            self.munchi_count += 1
            print(f"self.munchi_count: {self.munchi_count}")
            if (self.munchi_count // 10) < 3: 
                t.linear.x = -0.15 if self.munchi_count % 10 < 5 else 0.0
            else:
                self.state = "ACT2_WAIT"
                self.wait_start_time = now

        # 3단계: 대기 및 장애물 제거 확인
        elif self.state == "ACT2_WAIT":
            self.send_robot_cmd(face="suspicious", tail="stop") # 경계하는 표정으로 대기

            # 비전과 라이다 모두 장애물이 없다고 판단할 때
            if not has_obstacle:
                self.get_logger().info("✅ 장애물 제거 확인! 숫자를 보여주세요.")
                self.send_robot_cmd(face="happy", buzzer="happy", tail="friendly")
                
                # 중요: 장애물이 치워진 순간 이전의 숫자 데이터는 무시하도록 초기화
                self.current_digit = "none" 
                self.state = "ACT3_AUTHENTICATE" # 주인 인증 단계로!
                self.wait_start_time = now
                
            elif now - self.wait_start_time > 10.0: # 10초 무응답 시 우회
                self.get_logger().info("🔄 경로 우회 자율주행 모드 전환")
                self.state = "ACT2_BYPASS"
                self.munchi_count = 0

        # 우회 로직 - 여기에는 자율주행을 붙일 예정
        elif self.state == "ACT2_BYPASS":
            self.get_logger().info("우회 로직")
            self.send_robot_cmd(face="angry")
            self.munchi_count += 1
            if self.munchi_count < 20: t.angular.z = 1.2
            elif self.munchi_count < 60: t.linear.x = 0.15
            elif self.munchi_count < 80: t.angular.z = -1.2
            else: 
                self.send_robot_cmd(face="neutral", buzzer="happy")
                self.state = "ACT3_AUTHENTICATE"

        # [핵심 추가] 4. 주인 인증 (이리와!)
        elif self.state == "ACT3_AUTHENTICATE":
            if not is_stable:
                return
            self.get_logger().info("주인 인증")
            self.send_robot_cmd(face="veyes")
            print("current_gesture : ", self.current_gesture)
            print("current_face : ", self.current_face)
            if self.current_face == "manager" or self.current_gesture in ["브이", "보"]:
                self.get_logger().info("👋 주인님 확인 완료!")
                self.send_robot_cmd(face="greeting", buzzer="happy")
                self.state = "ACT4_DELIVERY"
            else:
                # 대기 중에는 가끔 눈을 깜박임
                if int(now) % 4 == 0: self.send_robot_cmd(face="blink")

        # 5. 숫자 인식 대기
        if self.state == "ACT4_DELIVERY":
            # 1. 시나리오 전환 후 2초 대기 (배경 인식 방지)
            if not is_stable:
                self.send_robot_cmd(face="blink")
                return

            # 2. 숫자가 2초 동안 똑같이 보여야 확정
            if self.is_value_confirmed("digit", self.current_digit, now, threshold=1.0):
                self.get_logger().info(f"✅ 숫자 {self.current_digit} 확정!")
                if self.current_digit in ["1", "3", "9"]:
                    self.move_duration = 3.0 if self.current_digit == "1" else 6.0 if self.current_digit == "3" else 9.0
                    self.send_robot_cmd(face="neutral")
                    self.state = "ACT4_MOVING"
                    self.wait_start_time = now
            else:
                self.get_logger().info(f"🔢 숫자 인식 중... ({self.current_digit})", once=True)

        # 6. 배달 이동 (자율주행)
        elif self.state == "ACT4_MOVING":
            self.get_logger().info("배달 이동 및 도착 회전")
            if now - self.wait_start_time < self.move_duration:
                t.linear.x = 0.02
            else:
                self.state = "ACT4_ARRIVED_SPIN"; self.wait_start_time = now

        # [추가된 로직] 4-2. 배달지 도착 후 5초간 회전
        elif self.state == "ACT4_ARRIVED_SPIN":
            self.get_logger().info("배달지 도착 후 5초간 회전")
            if now - self.wait_start_time < 5.0: # 5초 동안
                t.angular.z = 0.5 # 뺑글뺑글 회전
                self.send_robot_cmd(face="happy", tail="friendly")
            else:
                self.get_logger().info("🛑 회전 종료, 결제 대기")
                self.state = "ACT5_PAYMENT"

        # 7. 결제 및 쓰다듬기 상호작용
        elif self.state == "ACT5_PAYMENT":
            # 1. 안정화 대기 (상태 전환 후 2초간 무시)
            if not is_stable:
                self.send_robot_cmd(face="blink")
                return

            # [중요] C++ 노드와 라벨 이름을 반드시 맞추세요 (BLUE, GREEN, YELLOW)
            # 표정 매핑 딕셔너리 (코드 효율화)
            payment_config = {
                "GREEN":  {"face": "thankyou", "buzzer": "happy"},
                "YELLOW": {"face": "money",    "buzzer": "happy"},
                "BLUE":   {"face": "cry",      "buzzer": "warning"} # 천원(BLUE)은 울기
            }

            # 2. 색상이 2초 동안 유지되어야 인정
            if self.is_value_confirmed("money", self.last_obj, now, threshold=1.0):
                # 감지된 색상이 매핑 테이블에 있는지 확인
                if self.last_obj in payment_config:
                    cfg = payment_config[self.last_obj]
                    self.send_robot_cmd(face=cfg["face"], buzzer=cfg["buzzer"])
                    
                    # [주의] == 가 아니라 = 입니다! (상태 전환)
                    self.state = "ACT6_GREAT" 
                    self.get_logger().info(f"💰 결제 완료: {self.last_obj} -> 다음 단계로!")
            else:
                # 확정 전까지는 대기 표정
                self.send_robot_cmd(face="blink")

            

        elif self.state == "ACT6_GREAT":
            if not is_stable:
                return
            
            # 3. 엄지척도 2초는 유지되어야 댄스 시작
            if self.is_value_confirmed("gesture", self.current_gesture, now, threshold=1.0):
                if self.current_gesture == "엄지척":
                    self.send_robot_cmd(face="hearteye", buzzer="happy")
                    self.state = "ACT7_HAPPY_DANCE"
                    self.wait_start_time = now

        # 8. 행복한 댄스
        elif self.state == "ACT7_HAPPY_DANCE":
            self.send_robot_cmd(face="hearteye", tail="friendly")
            t.angular.z = 0.5
            if now - self.wait_start_time > 3.0:
                self.state = "ACT0_SLEEPY"

        self.cmd_pub.publish(t)

def main():
    rclpy.init()
    node = SdrMissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # Ctrl+C 호출 시 노드 로그에 깔끔하게 표시
        node.get_logger().info('Stopping the robot...')
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()