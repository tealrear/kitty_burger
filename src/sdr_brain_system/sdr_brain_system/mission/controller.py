import rclpy, time, json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

from .constants import *
from .utils import StateFilter, RobotCommandManager

class SdrMissionController(Node):
    def __init__(self):
        super().__init__('sdr_mission_controller')
        
        # 유틸리티 초기화
        self.filter = StateFilter()
        self.robot = RobotCommandManager(self)
        
        self.state = ACT0_SLEEPY
        self.last_state = None
        self.state_start_time = time.time()
        self._init_variables()
        self._init_comms()

        self.create_timer(0.1, self.main_loop)
        self.get_logger().info("🚀 [SDR] 미션 컨트롤러 가동")

    def _init_variables(self):
        self.last_obj, self.lidar_obstacle = "NONE", False
        self.current_face, self.current_gesture = "none", "none"
        self.current_digit, self.current_expression = "none", "none"
        self.munchi_count, self.wait_start_time, self.move_duration = 0, 0, 0

    def _init_comms(self):
        # 구독
        self.create_subscription(String, '/vision_fast_data', self.vision_cb, 10)
        self.create_subscription(String, '/person/digit', self.digit_cb, 10)
        self.create_subscription(String, '/person/hand', self.hand_cb, 10)
        self.create_subscription(String, '/person/face_id', self.face_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
        # 발행
        self.face_pub = self.create_publisher(String, '/face_cmd', QOS_MISSION)
        self.buzzer_pub = self.create_publisher(String, '/buzzer_cmd', QOS_MISSION)
        self.tail_pub = self.create_publisher(String, '/tail_cmd', QOS_MISSION)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QOS_MISSION)
        self.state_pub = self.create_publisher(String, '/mission_state', QOS_MISSION)

    def check_stable(self, duration):
        """현재 상태 진입 후 duration초가 지났는지 확인 + 로그 출력"""
        elapsed = time.time() - self.state_start_time
        remaining = duration - elapsed
        
        if remaining > 0:
            # 1초 간격으로만 남은 시간 로그 출력 (터미널 도배 방지)
            if int(elapsed * 10) % 10 == 0: 
                self.get_logger().info(f"⏳ [{self.state}] 안정화 대기 중... 남은 시간: {remaining:.1f}s")
            return False
        return True

    def main_loop(self):
        now = time.time()
        start_time = self.state_start_time

        t = Twist()

        # 상태 전환 감지
        if self.state != self.last_state:
            self.get_logger().info(f"🔄 상태 변경: {self.last_state} -> {self.state}")
            self.state_start_time = now # ROS 시간 객체 저장
            self.last_state = self.state
            self.last_obj, self.wait_start_time = "NONE", 0
            self.filter.timers.clear()
            self.robot.last_sent["buzzer"] = "" # 상태 바뀔 때 부저 초기화

        has_obstacle = (self.last_obj == "BLUE" or self.lidar_obstacle)
        self.state_pub.publish(String(data=self.state))

        # 1단계: 졸음 주행
        if self.state == ACT0_SLEEPY:
            self.get_logger().info("졸음 주행")
            if self.check_stable(7.0):
                self.robot.send(face="sleepy", tail="stop") # 꾸벅꾸벅 조는 표정
                t.linear.x = 0.01
                t.angular.z = 0.1 if int(now * 2) % 2 == 0 else -0.1

                # 장애물이 있으면
                if self.check_stable(3.0) and has_obstacle:
                    if now - self.state_start_time > 3.0:
                        self.get_logger().info("⚠️ 3초간 졸다가 장애물 발견! 후퇴 모드 진입")
                        self.state = ACT1_ALARM
                        self.munchi_count = 0
                    else:
                        pass
                

        # 2단계: 장애물 발견 및 3단 후퇴
        elif self.state == ACT1_ALARM:
            self.get_logger().info("장애물 발견 및 3단 후퇴")
            self.robot.send(face="surprise", buzzer="danger")
            self.munchi_count += 1
            if (self.munchi_count // 10) < 3: 
                t.linear.x = -0.1 if self.munchi_count % 10 < 5 else 0.0
            else:
                self.state = ACT2_WAIT
                self.wait_start_time = now

        # 3단계: 대기 및 장애물 제거 확인
        elif self.state == ACT2_WAIT:
            self.get_logger().info("대기 및 장애물 제거 확인")
            self.robot.send(face="suspicious", tail="stop") # 경계하는 표정으로 대기

            # 비전과 라이다 모두 장애물이 없다고 판단할 때
            if not has_obstacle:
                self.get_logger().info("✅ 장애물 제거 확인! 주인님 보여주세요.")
                self.robot.send(face="happy", buzzer="happy")
                
                # 중요: 장애물이 치워진 순간 이전의 숫자 데이터는 무시하도록 초기화
                self.current_digit = "none" 
                self.state = ACT3_AUTHENTICATE # 주인 인증 단계로!
                self.wait_start_time = now

                
            elif now - self.wait_start_time > 10.0: # 10초 무응답 시 우회
                self.get_logger().info("🔄 경로 우회 자율주행 모드 전환")
                self.state = ACT2_BYPASS
                self.munchi_count = 0

        # 우회 로직 - 여기에는 자율주행을 붙일 예정
        elif self.state == ACT2_BYPASS:
            self.get_logger().info("자율주행")
            self.robot.send(face="angry", buzzer="warning")
            self.munchi_count += 1
            if self.munchi_count < 20: t.angular.z = 0.01
            elif self.munchi_count < 60: t.linear.x = 0.01
            elif self.munchi_count < 80: t.angular.z = -0.01
            else: 
                self.robot.send(face="neutral", buzzer="happy")
                self.state = ACT3_AUTHENTICATE

        # [핵심 추가] 4. 주인 인증 (이리와!)
        elif self.state == ACT3_AUTHENTICATE:
            if self.check_stable(5.0):
                self.get_logger().info("주인 인증 (이리와!)")
                self.robot.send(face="veyes")
                print("current_gesture : ", self.current_gesture)
                print("current_face : ", self.current_face)

                # 아직 인증 성공 시간을 기록하지 않은 경우 (인증 시도 중)
                if self.wait_start_time == 0:
                    if self.current_face == "manager" or self.current_gesture in ["브이", "보"]:
                        self.get_logger().info("👋 주인님 확인 완료! 2초간 인사합니다.")
                        self.robot.send(face="greeting", buzzer="happy") # "heart" 표정 전송
                        self.wait_start_time = now  # 성공한 시점의 시간 기록
                
                # 인증 성공 시간을 기록했다면 (인증 완료 후 대기 중)
                else:
                    # 2초 동안은 greeting 표정을 유지하며 대기
                    if now - self.wait_start_time > 2.0:
                        self.get_logger().info("🚚 배달 단계로 이동합니다.")
                        self.state = ACT4_DELIVERY

        # 5. 숫자 인식 대기
        if self.state == ACT4_DELIVERY:
            # 1. 시나리오 전환 후 2초 대기 (배경 인식 방지)
            if self.check_stable(5.0):
                # 2. 숫자가 1초 동안 똑같이 보여야 확정
                if self.filter.is_confirmed("digit", self.current_digit, threshold=2.0):
                    self.get_logger().info(f"✅ 숫자 {self.current_digit} 확정!")
                    if self.current_digit in ["1", "3", "9"]:
                        self.move_duration = 3.0 if self.current_digit == "1" else 6.0 if self.current_digit == "3" else 9.0
                        self.robot.send(face="blink", buzzer="warning")
                        self.state = ACT4_MOVING
                        self.wait_start_time = now
                else:
                    self.get_logger().info(f"🔢 숫자 인식 중... ({self.current_digit})", once=True)

        # 6. 배달 이동 (자율주행)
        elif self.state == ACT4_MOVING:
            self.get_logger().info("배달 이동 및 도착 회전")
            if now - self.wait_start_time < self.move_duration:
                t.linear.x = 0.02
            else:
                self.state = ACT4_ARRIVED_SPIN; self.wait_start_time = now

        # [추가된 로직] 4-2. 배달지 도착 후 5초간 회전
        elif self.state == ACT4_ARRIVED_SPIN:
            self.get_logger().info("배달지 도착 후 5초간 회전")
            if now - self.wait_start_time < 5.0: # 5초 동안
                t.angular.z = 0.1 # 뺑글뺑글 회전
                self.robot.send(face="happy", buzzer="happy")
            else:
                self.get_logger().info("🛑 회전 종료, 결제 대기")
                self.state = ACT5_PAYMENT

        # 7. 결제 및 쓰다듬기 상호작용
        elif self.state == ACT5_PAYMENT:
            self.get_logger().info(f"결제 및 쓰다듬기 상호작용")

            # 1. 안정화 대기 (상태 전환 후 2초간 무시)
            if self.check_stable(7.0):
                # [중요] C++ 노드와 라벨 이름을 반드시 맞추세요 (BLUE, GREEN, YELLOW)
                # 표정 매핑 딕셔너리 (코드 효율화)
                payment_config = {
                    "MONEY_RED":  {"face": "thankyou", "buzzer": "happy"},
                    "MONEY_GREEN": {"face": "money",    "buzzer": "happy"},
                    "MONEY_BLUE":   {"face": "cry",      "buzzer": "warning"} # 천원(BLUE)은 울기
                }

                # 2. 색상이 2초 동안 유지되어야 인정

                if self.filter.is_confirmed("money", self.last_obj, threshold=2.0):
                    if self.last_obj in payment_config:
                        cfg = payment_config[self.last_obj]
                        self.robot.send(face=cfg["face"], buzzer=cfg["buzzer"])
                        self.state = ACT6_GREAT
                        self.get_logger().info(f"💰 결제 완료: {self.last_obj} -> 다음 단계로!")
                else: self.robot.send(face="blink")

        elif self.state == ACT6_GREAT:
            self.get_logger().info(f"주인님 엄지척")
            if self.check_stable(5.0):
                if self.current_gesture == "엄지척":
                    self.robot.send(face="hearteye", buzzer="happy")
                    self.state = ACT7_HAPPY_DANCE
                    self.wait_start_time = now  

        # 8. 행복한 댄스
        elif self.state == ACT7_HAPPY_DANCE:
            self.get_logger().info(f"행복한 댄스")
            self.robot.send(face="hearteye", tail="friendly", buzzer="happy")
            
            # 0.1초마다 방향을 바꿔서 빠르게 흔들기 (도리도리)
            t.angular.z = 1.5 if int(now * 10) % 2 == 0 else -1.5
            
            # 0.2초마다 앞뒤로 움직여서 들썩거리기 (위아래 느낌)
            t.linear.x = 0.05 if int(now * 5) % 2 == 0 else -0.05
            
            if now - self.wait_start_time > 5.0:
                self.get_logger().info("🏁 댄스 종료, 다시 졸음 주행으로...")
                self.state = ACT0_SLEEPY

        self.cmd_pub.publish(t)

    # 콜백 함수들... (기존과 동일)
    def vision_cb(self, msg): self.last_obj = msg.data.split(':')[0]
    def digit_cb(self, msg): self.current_digit = msg.data
    def hand_cb(self, msg):
        try: self.current_gesture = json.loads(msg.data).get("gesture", "none")
        except: pass
    def face_cb(self, msg): self.current_face = msg.data
    def lidar_cb(self, msg):
        front = msg.ranges[0:15] + msg.ranges[-15:]
        valid = [r for r in front if r > 0.05]
        if valid: self.lidar_obstacle = min(valid) < LIDAR_SAFE_DISTANCE

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SdrMissionController())
    rclpy.shutdown()