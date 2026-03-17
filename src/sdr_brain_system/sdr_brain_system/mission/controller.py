import rclpy, time, json, math
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
        remaining = duration - self.elapsed
        
        if remaining > 0:
            # 1초 간격으로만 남은 시간 로그 출력 (터미널 도배 방지)
            if int(self.elapsed * 10) % 10 == 0: 
                self.get_logger().info(f"⏳ [{self.state}] 안정화 대기 중... 남은 시간: {remaining:.1f}s")
            return False
        return True
    
    @property
    def elapsed(self):
        """현재 상태에 진입한 후 경과된 시간(초)을 반환"""
        return time.time() - self.state_start_time

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
            self.munchi_count = 0

        has_obstacle = (self.last_obj == "BLUE" or self.lidar_obstacle)
        self.state_pub.publish(String(data=self.state))

        # 1단계: 졸음 주행
        if self.state == ACT0_SLEEPY:
            self.get_logger().info("졸음 주행")
            if self.check_stable(17.0):
                self.robot.send(face="sleepy", tail="stop") # 꾸벅꾸벅 조는 표정
                t.linear.x = 0.01
                t.angular.z = 0.1 if int(now * 2) % 2 == 0 else -0.1

                # 장애물이 있으면
                if has_obstacle:
                    self.get_logger().info("⚠️ 3초간 졸다가 장애물 발견! 후퇴 모드 진입")
                    self.state = ACT1_ALARM
                    self.munchi_count = 0
                

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

            # 1. 장애물이 없는 상태가 2.0초 동안 지속되는지 확인합니다.
            # "obstacle_status"라는 카테고리로 현재 장애물 유무를 문자열로 필터링합니다.
            obstacle_label = "EXIST" if has_obstacle else "NONE"
            is_clear_confirmed = self.filter.is_confirmed("obstacle_status", obstacle_label, threshold=2.0)

            if int(now * 5) % 10 == 0: # 2초마다 로그 출력
                self.get_logger().info(f"👀 장애물 확인 중... 상태: {obstacle_label}")

            self.robot.send(face="suspicious", tail="stop") # 경계하는 표정으로 대기

            # 2. 단순히 'not has_obstacle'이 아니라, 'NONE' 상태가 2초간 확정되었을 때만 통과!
            if is_clear_confirmed and obstacle_label == "NONE":
                self.get_logger().info("✅ 장애물 제거 확인! 주인님 보여주세요.")
                self.robot.send(face="happy", buzzer="happy")
                
                self.current_digit = "none" 
                self.state = ACT3_AUTHENTICATE 
                self.wait_start_time = now
                
            elif self.elapsed > 5.0: # 5초 무응답 시 우회
                self.get_logger().info("🔄 경로 우회 자율주행 모드 전환")
                self.state = ACT2_BYPASS
            
        # 우회 로직 - 여기에는 자율주행을 붙일 예정
        elif self.state == ACT2_BYPASS:
            self.get_logger().info("자율주행")
            self.get_logger().info(f"🔄 자율주행 우회 중... ({self.elapsed:.1f}s)")
            self.robot.send(face="angry", buzzer="warning")

            self.munchi_count += 1
            if self.elapsed < 2.0: t.angular.z = 0.5       # 2초간 회전
            elif self.elapsed < 5.0: t.linear.x = 0.15     # 3초간 전진
            elif self.elapsed < 7.0: t.angular.z = -0.5    # 2초간 반대 회전
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

                # 1. 제스처 2초 확정 검사
                is_gesture_ok = self.filter.is_confirmed("gesture", self.current_gesture, threshold=2.0)
                
                # 2. 얼굴 인식 1초 확정 검사 (새로 추가된 부분)
                is_face_confirmed = self.filter.is_confirmed("face", self.current_face, threshold=1.0)

                # 아직 인증 성공 시간을 기록하지 않은 경우 (인증 시도 중)
                if self.wait_start_time == 0:
                    if (is_face_confirmed and self.current_face == "manager") or (is_gesture_ok and self.current_gesture == "브이"):
                        self.get_logger().info(f"👋 주인님 확인 완료! (방법: {'얼굴' if is_face_confirmed else '제스처'})")
                        self.robot.send(face="greeting", buzzer="happy") # "heart" 표정 전송
                        self.wait_start_time = now  # 성공한 시점의 시간 기록
                
                # 인증 성공 시간을 기록했다면 (인증 완료 후 대기 중)
                else:
                    # 2초 동안은 greeting 표정을 유지하며 대기
                    if self.elapsed > 2.0:
                        self.get_logger().info("🚚 배달 단계로 이동합니다.")
                        self.state = ACT4_DELIVERY

        # 5. 숫자 인식 대기
        if self.state == ACT4_DELIVERY:
            # 1. 시나리오 전환 후 5초 대기 (배경 인식 방지)
            if self.check_stable(5.0):
                # 2. 숫자가 1초 동안 똑같이 보여야 확정
                if self.filter.is_confirmed("digit", self.current_digit, threshold=2.0):
                    self.get_logger().info(f"✅ 숫자 {self.current_digit} 확정!")
                    if self.current_digit in ["1", "3", "9"]:

                        durations = {"1": 5.0, "3": 10.0, "9": 15.0}
                        self.move_duration = durations[self.current_digit]

                        self.robot.send(face="blink", buzzer="warning")
                        self.state = ACT4_MOVING
                        self.wait_start_time = now
                else:
                    self.get_logger().info(f"🔢 숫자 인식 중... ({self.current_digit})", once=True)

        # 6. 배달 이동 (자율주행)
        elif self.state == ACT4_MOVING:
            self.get_logger().info("배달 이동 및 도착 회전")
            if self.elapsed < self.move_duration:
                self.get_logger().info(f"🚚 배달 전진 중... ({self.elapsed:.1f}/{self.move_duration}s)")
                t.linear.x = 0.05 # 전진 속도 상향 (0.02 -> 0.12)
            else:
                self.state = ACT4_ARRIVED_SPIN

        # [추가된 로직] 4-2. 배달지 도착 후 5초간 회전
        elif self.state == ACT4_ARRIVED_SPIN:
            if self.elapsed < 5.0: 
                self.get_logger().info(f"💃 도착 기념 회전! ({self.elapsed:.1f}s)")
                t.angular.z = 1.2 # 회전 속도 상향
                self.robot.send(face="happy", buzzer="happy")
            else:
                self.get_logger().info("🛑 회전 종료, 결제 대기")
                self.state = ACT5_PAYMENT

        # 7. 결제 및 상호작용 (2초 보자기 -> 7초 대기 -> 돈 인식)
        elif self.state == ACT5_PAYMENT:
            # 1. 보자기 2초 인식 확인
            is_bo_confirmed = self.filter.is_confirmed("gesture", self.current_gesture, threshold=2.0)

            # [Step 1] 아직 보자기 2초 확인 전
            if self.wait_start_time == 0:
                if is_bo_confirmed and self.current_gesture == "보":
                    self.get_logger().info("🖐️ 보자기 2초 확인! 7초간 돈 준비를 기다립니다.")
                    self.wait_start_time = now  # 7초 타이머 시작 시점 기록
                    self.robot.send(face="happy", buzzer="happy") # "알겠냥!" 하는 반응
                else:
                    # 보자기 보여달라고 유도하는 단계
                    if int(now * 5) % 10 == 0:
                        self.get_logger().info("✋ 결제를 위해 손바닥(보)을 2초간 보여주세요...", once=True)
                    self.robot.send(face="neutral")

            # [Step 2 & 3] 보자기가 확인되어 7초 타이머가 돌아가는 중
            else:
                # [Step 2] 7초가 아직 안 지남 (돈 준비 시간)
                if self.elapsed < 7.0:
                    remaining = 7.0 - self.elapsed
                    if int(self.elapsed * 10) % 10 == 0: # 1초마다 로그
                        self.get_logger().info(f"⏳ 돈 준비를 기다리는 중... 남은 시간: {remaining:.1f}s")
                    
                    # 로봇이 기대하며 눈을 깜빡거림
                    self.robot.send(face="blink") 

                # [Step 3] 드디어 7초 경과! 이제부터 진짜 돈 색상을 감별함
                else:
                    self.get_logger().info("🔍 7초 경과! 이제 돈 색상을 인식합니다.")
                    is_money_confirmed = self.filter.is_confirmed("money", self.last_obj, threshold=1.5)

                    payment_config = {
                        "MONEY_RED":  {"face": "thankyou", "buzzer": "happy", "name": "오천원"},
                        "MONEY_GREEN": {"face": "money",    "buzzer": "happy", "name": "만원"},
                        "MONEY_BLUE":   {"face": "cry",      "buzzer": "warning", "name": "천원"}
                    }

                    if is_money_confirmed and self.last_obj in payment_config:
                        cfg = payment_config[self.last_obj]
                        self.get_logger().info(f"💰 결제 완료: {cfg['name']}! 주인님 엄지척 해주세요.")
                        self.robot.send(face=cfg["face"], buzzer=cfg["buzzer"])
                        self.state = ACT6_GREAT # 다음 단계로 이동
                    else:
                        # 돈이 아직 안 보이거나 인식이 안 될 때
                        self.robot.send(face="smile") # 웃으며 계속 기다림

        elif self.state == ACT6_GREAT:
            self.get_logger().info(f"주인님 엄지척")

            # 제스처 2초 확정 검사
            is_gesture_ok = self.filter.is_confirmed("gesture", self.current_gesture, threshold=2.0)

            if self.check_stable(5.0):
                if is_gesture_ok and self.current_gesture == "엄지척":
                    self.get_logger().info("👍 엄지척 확인! 행복한 댄스 시작!")
                    self.robot.send(face="hearteye", buzzer="happy")
                    self.state = ACT7_HAPPY_DANCE
                    self.wait_start_time = now  

        # 8. 행복한 댄스
        elif self.state == ACT7_HAPPY_DANCE:
            self.get_logger().info(f"🐱 냥이 둠칫둠칫 중... ({self.elapsed:.1f}s)")
            self.robot.send(face="hearteye", tail="friendly", buzzer="happy")

            if self.elapsed < 4.0:
                # [Step 1] 실룩실룩 모드 (약 4초간)
                # 터틀봇 버거 권장 최대 회전 속도는 약 2.8 rad/s입니다. 2.5 정도로 조절할게요!
                t.angular.z = 2.5 * math.sin(now * 15) # 좌우로 빠르게 도리도리
                t.linear.x = 0.05 * math.sin(now * 8)  # 앞뒤로 살짝 들썩들썩
            
            elif self.elapsed < 6.0:
                # [Step 2] 기분 최고! 빙그르르 스핀 (2초간)
                t.linear.x = 0.0
                t.angular.z = 2.0 # 안전하게 신나는 속도
                
            else:
                # [Step 3] 댄스 종료
                self.get_logger().info("🏁 댄스 완료! 냥이는 이제 졸려요...")
                t.linear.x = 0.0
                t.angular.z = 0.0
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
        # 전방 50도 (왼쪽 25도 + 오른쪽 25도)
        # msg.ranges[0:25] -> 0도 ~ 24도
        # msg.ranges[-25:] -> 335도 ~ 359도
        front = msg.ranges[0:25] + msg.ranges[-25:]
        
        # 0.05m(5cm)보다 큰 유효한 거리 값만 필터링 (센서 노이즈 제거)
        valid = [r for r in front if r > 0.05]
        
        if valid:
            # 유효 거리 중 최솟값이 안전 거리(LIDAR_SAFE_DISTANCE)보다 작으면 장애물로 판단
            self.lidar_obstacle = min(valid) < LIDAR_SAFE_DISTANCE

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SdrMissionController())
    rclpy.shutdown()