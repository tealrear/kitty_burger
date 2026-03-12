import rclpy, time, json
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SdrMissionController(Node):
    def __init__(self):
        super().__init__('sdr_mission_controller')
        
        self.state = "ACT1_NAV2"
        self.last_seen_time = time.time()
        self.act2_start_time = None
        
        # 애니메이션 제어용 카운터
        self.munchi_step = 0 
        
        self.current_gesture = "none"
        self.current_expression = "none"
        
        self.create_subscription(String, '/vision_fast_data', self.vision_cb, 10)
        self.create_subscription(String, '/person/hand', self.hand_cb, 10)
        self.create_subscription(String, '/person/expression', self.exp_cb, 10)
        
        self.face_pub = self.create_publisher(String, '/face_lcd', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 0.1초마다 실행되는 메인 루프
        self.create_timer(0.1, self.drive_loop)
        
        self.send_face("blink")
        self.get_logger().info("👑 [SDR] 부기 컨트롤러 가동 - 후퇴 로직 보정 완료")

    def hand_cb(self, msg):
        try: self.current_gesture = json.loads(msg.data).get("gesture", "none")
        except: pass

    def exp_cb(self, msg):
        self.current_expression = msg.data

    def vision_cb(self, msg):
        data = msg.data.split(':')
        obj_type, cx = data[0], int(data[1]) if len(data) > 1 else 0
        curr_time = time.time()

        # 🎬 1막: 장애물 발견 시 상태 즉시 변경
        if self.state == "ACT1_NAV2" and obj_type == "BLUE":
            self.get_logger().warn("🚨 [1막] 장애물 발견! 알람 상태로 전환")
            self.state = "ACT1_ALARM" # 후퇴 애니메이션 상태
            self.munchi_step = 0      # 카운터 초기화
            self.act2_start_time = curr_time
            self.send_face("surprised")

        # 🎬 2막 이후 로직 (기존과 동일)
        elif self.state == "ACT2_TRACKING":
            if obj_type == "BLUE":
                if int(curr_time) % 5 == 0: self.get_logger().info("🎵 치워주세요!")
                if curr_time - self.act2_start_time > 10.0:
                    self.send_face("angry"); self.state = "ACT1_NAV2"
            elif obj_type == "MASTER":
                self.track_person(cx)
                if self.current_gesture in ["브이", "엄지척"] or self.current_expression == "happy":
                    self.send_face("heart"); self.state = "ACT3_FOLLOWING"
            else: self.publish_twist(0.0, 0.3) # 탐색 회전

    # --- 🔄 메인 드라이브 루프 (여기서 모든 이동 제어) ---
    def drive_loop(self):
        t = Twist()

        if self.state == "ACT1_NAV2":
            # 일반 주행
            t.linear.x = 0.08 # 속도를 조금 높임 (너무 낮으면 안 움직임)

        elif self.state == "ACT1_ALARM":
            # [멈칫-후퇴-회전] 애니메이션 (0.1초 단위)
            self.munchi_step += 1
            
            if self.munchi_step <= 5: # 0.5초간 정지 (멈칫)
                t.linear.x = 0.0
            elif self.munchi_step <= 20: # 1.5초간 후퇴
                t.linear.x = -0.15 # 확실히 뒤로 빼기
            elif self.munchi_step <= 35: # 1.5초간 제자리 회전
                t.angular.z = 1.5
                self.send_face("angry")
            else:
                # 애니메이션 종료 후 2막으로 전환
                self.get_logger().info("✅ 후퇴 완료, 주인님 대기 모드")
                self.state = "ACT2_TRACKING"
                self.munchi_step = 0

        elif self.state == "ACT3_FOLLOWING":
            # 3막은 vision_cb나 별도 로직에서 처리하므로 여기선 pass 가능
            return 

        self.cmd_pub.publish(t)

    def publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.cmd_pub.publish(t)

    def track_person(self, cx):
        if cx < 120: self.send_face("look_left")
        elif cx > 200: self.send_face("look_right")
        else: self.send_face("look_center")

    def send_face(self, cmd):
        self.face_pub.publish(String(data=cmd))

def main():
    rclpy.init(); rclpy.spin(SdrMissionController()); rclpy.shutdown()