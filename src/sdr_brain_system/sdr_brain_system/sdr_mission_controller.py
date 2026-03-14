import rclpy, time, json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan # 라이다 사용을 위한 임포트
from rclpy.qos import qos_profile_sensor_data # 상단에 추가

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

        # 현재 상태 브로드캐스팅
        self.state_pub.publish(String(data=self.state))

        # 장애물 여부 종합 (비전에서 파란색 감지 OR 라이다에서 근접 물체 감지)
        has_obstacle = (self.last_obj == "BLUE" and self.lidar_obstacle)
        print(f"Lidar: {self.lidar_obstacle}, Vision: {self.last_obj}, Combined: {has_obstacle}")


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
            self.send_robot_cmd(face="surprise", buzzer="warning", tail="angry")
            self.munchi_count += 1
            print(f"self.munchi_count: {self.munchi_count}")
            if (self.munchi_count // 10) < 3: 
                t.linear.x = -0.15 if self.munchi_count % 10 < 5 else 0.0
            else:
                self.state = "ACT2_WAIT"
                self.wait_start_time = now
                self.send_robot_cmd(face="suspicious", tail="stop") # 경계하는 표정으로 대기

        # 3단계: 대기 및 장애물 제거 확인
        elif self.state == "ACT2_WAIT":
            self.send_robot_cmd(face="suspicious") # 아직 경계 중

            # 비전과 라이다 모두 장애물이 없다고 판단할 때
            if not has_obstacle:
                self.get_logger().info("✅ 장애물 제거 확인! 숫자를 보여주세요.")
                self.send_robot_cmd(face="greeting", buzzer="happy", tail="friendly")
                
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
            self.munchi_count += 1
            if self.munchi_count < 20: t.angular.z = 1.2
            elif self.munchi_count < 60: t.linear.x = 0.15
            elif self.munchi_count < 80: t.angular.z = -1.2
            else: 
                self.send_robot_cmd(face="neutral", buzzer="happy")
                self.state = "ACT3_AUTHENTICATE"

        # [핵심 추가] 4. 주인 인증 (이리와!)
        elif self.state == "ACT3_AUTHENTICATE":
            self.get_logger().info("주인 인증")
            self.send_face("suspicious") # "누구지?" 하는 궁금한 표정
            print("current_gesture : ", self.current_gesture)
            print("current_face : ", self.current_face)
            if self.current_face == "manager" or self.current_gesture in ["엄지척", "보"]:
                self.get_logger().info("👋 주인님 확인! 반가워요!")
                self.send_robot_cmd(face="hearteye", buzzer="happy", tail="friendly")
                self.state = "ACT4_DELIVERY"
                self.current_digit = "none" # 숫자 데이터 초기화
            else:
                # 대기 중에는 가끔 눈을 깜박임
                if int(now) % 4 == 0: self.send_robot_cmd(face="blink")

        # 5. 숫자 인식 대기
        elif self.state == "ACT4_DELIVERY":
            if self.current_digit in ["1", "3", "9"]:
                self.move_duration = 3.0 if self.current_digit == "1" else 6.0 if self.current_digit == "3" else 9.0
                self.send_robot_cmd(face="veyes", buzzer="happy")
                self.state = "ACT4_MOVING"; self.wait_start_time = now

        # 6. 배달 이동
        elif self.state == "ACT4_MOVING":
            self.get_logger().info("배달 이동 및 도착 회전")
            if now - self.wait_start_time < self.move_duration:
                t.linear.x = 0.12
                if int(now * 2) % 2 == 0: self.send_robot_cmd(face="blink")
            else:
                self.state = "ACT4_ARRIVED_SPIN"; self.wait_start_time = now

        # [추가된 로직] 4-2. 배달지 도착 후 5초간 회전
        elif self.state == "ACT4_ARRIVED_SPIN":
            self.get_logger().info("배달지 도착 후 5초간 회전")
            if now - self.wait_start_time < 5.0: # 5초 동안
                t.angular.z = 1.5 # 뺑글뺑글 회전
                self.send_robot_cmd(face="veyes", tail="friendly") # 회전하는 동안 윙크 표정
            else:
                self.get_logger().info("🛑 회전 종료, 결제 대기")
                self.state = "ACT5_PAYMENT"
                self.send_robot_cmd(face="money", tail="stop")

        # 7. 결제 및 쓰다듬기 상호작용
        elif self.state == "ACT5_PAYMENT":
            self.get_logger().info("결제 및 쓰다듬기 상호작용")
            # 돈 확인 로직
            if self.last_obj == "GREEN": self.send_face("thankyou")
            elif self.last_obj == "YELLOW": self.send_face("money")
            else: self.send_face("wink")
            
            # 쓰다듬기 감지
            if self.current_gesture == "쓰다듬기":
                self.get_logger().info("🥰 주인님의 손길! 행복해요!")
                self.send_face("hearteye")
                self.state = "ACT6_HAPPY_DANCE"; self.wait_start_time = now

        # 8. 행복한 댄스
        elif self.state == "ACT6_HAPPY_DANCE":
            self.send_robot_cmd(face="hearteye", tail="friendly")
            t.angular.z = 2.5
            if now - self.wait_start_time > 3.0:
                self.state = "ACT0_SLEEPY"

        self.cmd_pub.publish(t)

    def send_face(self, cmd): self.face_pub.publish(String(data=cmd))
    def send_sound(self, cmd): self.sound_pub.publish(String(data=cmd))

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