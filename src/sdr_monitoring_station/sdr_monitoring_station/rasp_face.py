import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
from RPLCD.i2c import CharLCD
from time import sleep
from gpiozero import AngularServo, TonalBuzzer
from gpiozero.tones import Tone
import threading

# =========================
# LCD / BUZZER 전역 장치
# =========================
lcd = CharLCD('PCF8574', address=0x27, port=1, cols=16, rows=2)
buzzer = TonalBuzzer(19)   # BCM GPIO19

# =========================
# 공통 함수
# =========================
def load_chars(chars):
    for slot, pattern in chars.items():
        lcd.create_char(slot, pattern)

def clear():
    lcd.clear()

def booting_screen():
    lcd.clear()
    lcd.cursor_pos = (0, 3)
    lcd.write_string("BOOTING")
    for i in range(6):
        lcd.cursor_pos = (1, 5)
        lcd.write_string("." * ((i % 3) + 1) + "   ")
        sleep(0.4)

# =========================
# 1) 귀여운 얼굴 + 눈깜박임
# =========================
def load_cute_face():
    EYE_OPEN = [
        0b01110, 0b10001, 0b10101, 0b10001,
        0b01110, 0b00000, 0b00000, 0b00000,
    ]
    EYE_CLOSED = [
        0b00000, 0b00000, 0b11111, 0b01110,
        0b00000, 0b00000, 0b00000, 0b00000,
    ]
    MOUTH_SMILE_L = [
        0b00000, 0b00000, 0b00000, 0b00000,
        0b10000, 0b01000, 0b00110, 0b00001,
    ]
    MOUTH_SMILE_R = [
        0b00000, 0b00000, 0b00000, 0b00000,
        0b00001, 0b00010, 0b01100, 0b10000,
    ]

    load_chars({
        0: EYE_OPEN,
        1: EYE_CLOSED,
        2: MOUTH_SMILE_L,
        3: MOUTH_SMILE_R,
    })

def draw_cute(open_eye=True):
    eye = 0 if open_eye else 1
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(eye))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(eye))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(2) + chr(3))

def show_blink(seconds=5):
    load_cute_face()
    elapsed = 0.0
    while elapsed < seconds:
        draw_cute(True)
        sleep(0.8)
        elapsed += 0.8
        if elapsed >= seconds:
            break
        draw_cute(False)
        sleep(0.15)
        elapsed += 0.15

# =========================
# 2) 화난 얼굴
# =========================
def load_angry_face():
    EYE_ANGRY_L = [
        0b11100, 0b11100, 0b00110, 0b00111,
        0b00011, 0b00001, 0b00000, 0b00000,
    ]
    EYE_ANGRY_R = [
        0b00111, 0b00111, 0b00110, 0b11100,
        0b11000, 0b10000, 0b00000, 0b00000,
    ]
    MOUTH_ANGRY_L = [
        0b00000, 0b00000, 0b00110, 0b01000,
        0b10000, 0b00000, 0b00000, 0b00000,
    ]
    MOUTH_ANGRY_R = [
        0b00000, 0b00000, 0b01100, 0b00010,
        0b00001, 0b00000, 0b00000, 0b00000,
    ]

    load_chars({
        0: EYE_ANGRY_L,
        1: EYE_ANGRY_R,
        2: MOUTH_ANGRY_L,
        3: MOUTH_ANGRY_R,
    })

def show_angry(seconds=5):
    load_angry_face()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(1))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(2) + chr(3))
    sleep(seconds)

# =========================
# 3) 무표정
# =========================
def load_neutral_face():
    EYE_NEUTRAL = [
        0b00000, 0b11111, 0b11111, 0b00000,
        0b00000, 0b00000, 0b00000, 0b00000,
    ]
    MOUTH_FLAT = [
        0b00000, 0b00000, 0b11111, 0b11111,
        0b00000, 0b00000, 0b00000, 0b00000,
    ]

    load_chars({
        0: EYE_NEUTRAL,
        1: MOUTH_FLAT,
    })

def show_neutral(seconds=5):
    load_neutral_face()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1))
    sleep(seconds)

# =========================
# 4) 우는 얼굴
# =========================
def load_cry_face():
    EYE_SAD = [
        0b01110, 0b10000, 0b10100, 0b10010,
        0b01111, 0b00000, 0b00000, 0b00000,
    ]
    MOUTH_CRY_L = [
        0b00000, 0b00000, 0b00110, 0b01000,
        0b10000, 0b00000, 0b00000, 0b00000,
    ]
    MOUTH_CRY_R = [
        0b00000, 0b00000, 0b01100, 0b00010,
        0b00001, 0b00000, 0b00000, 0b00000,
    ]

    load_chars({
        0: EYE_SAD,
        1: MOUTH_CRY_L,
        2: MOUTH_CRY_R,
    })

def draw_cry(tears_left=False, tears_right=False):
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1) + chr(2))

    if tears_left:
        lcd.cursor_pos = (1, 4)
        lcd.write_string(".")
    if tears_right:
        lcd.cursor_pos = (1, 11)
        lcd.write_string(".")

def show_cry(seconds=5):
    load_cry_face()
    elapsed = 0.0
    while elapsed < seconds:
        draw_cry(True, False)
        sleep(0.4)
        elapsed += 0.4
        if elapsed >= seconds:
            break
        draw_cry(False, True)
        sleep(0.4)
        elapsed += 0.4
        if elapsed >= seconds:
            break
        draw_cry(False, False)
        sleep(0.3)
        elapsed += 0.3

# =========================
# 5) 메시지 아이콘
# =========================
def load_message_icon():
    MAIL_TOP = [
        0b11111, 0b10001, 0b01010, 0b00100,
        0b00100, 0b00000, 0b00000, 0b00000,
    ]
    MAIL_BOTTOM = [
        0b00000, 0b00000, 0b10001, 0b11011,
        0b11111, 0b11111, 0b00000, 0b00000,
    ]

    load_chars({
        0: MAIL_TOP,
        1: MAIL_BOTTOM,
    })

def show_message(seconds=5):
    load_message_icon()
    lcd.clear()
    lcd.cursor_pos = (0, 7)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1))
    lcd.cursor_pos = (0, 2)
    lcd.write_string("MSG")
    sleep(seconds)

# =========================
# 6) 하트 아이콘
# =========================
def load_heart_icon():
    HEART = [
        0b00000, 0b01010, 0b11111, 0b11111,
        0b11111, 0b01110, 0b00100, 0b00000,
    ]
    load_chars({0: HEART})

def show_heart(seconds=5):
    load_heart_icon()
    lcd.clear()
    lcd.cursor_pos = (0, 7)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 4)
    lcd.write_string("LOVE")
    sleep(seconds)

# =========================
# 7) 놀란 표정
# =========================
def load_surprised_face():
    BIG_EYE = [
        0b01110,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b10001,
        0b01110,
        0b00000,
    ]
    MOUTH_O = [
        0b00000,
        0b00000,
        0b00100,
        0b01010,
        0b01010,
        0b01010,
        0b00100,
        0b00000,
    ]

    load_chars({
        0: BIG_EYE,
        1: MOUTH_O,
    })

def show_surprised(seconds=5):
    load_surprised_face()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1))
    sleep(seconds)


# =========================
# 8) 당황스러운 표정
# =========================

def load_suspicious_eyes(seconds=5):
    EYE_LEFT = [
        0b11111,
        0b10001,
        0b11001,
        0b11001,
        0b11001,
        0b10001,
        0b11111,
        0b00000,
    ]
    EYE_CENTER = [
        0b11111,
        0b10001,
        0b10101,
        0b10101,
        0b10101,
        0b10001,
        0b11111,
        0b00000,
    ]
    EYE_RIGHT = [
        0b11111,
        0b10001,
        0b10011,
        0b10011,
        0b10011,
        0b10001,
        0b11111,
        0b00000,
    ]
    MOUTH_FLAT = [
        0b00000,
        0b00000,
        0b00000,
        0b11111,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    ]

    load_chars({
        0: EYE_LEFT,
        1: EYE_CENTER,
        2: EYE_RIGHT,
        3: MOUTH_FLAT,
    })

def draw_suspicious(pos=1):
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(pos))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(pos))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(3))

def show_suspicious(seconds=5):
    load_suspicious_eyes()
    frames = [0, 1, 2, 1]
    elapsed = 0.0
    while elapsed < seconds:
        for f in frames:
            draw_suspicious(f)
            sleep(0.25)
            elapsed += 0.25
            if elapsed >= seconds:
                break

# =========================
# 9) V눈  표정
# =========================

def load_v_eyes():
    V_EYE = [
        0b10001,
        0b10001,
        0b01010,
        0b01010,
        0b00100,
        0b00100,
        0b00000,
        0b00000,
    ]
    SMILE = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b10001,
        0b01010,
        0b00100,
        0b00000,
    ]
    load_chars({
        0: V_EYE,
        1: SMILE,
    })

def v_eyes(seconds=5):
    load_v_eyes()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1))
    sleep(seconds)


# =========================
# 10)하트눈 표정
# =========================


def load_heart_eyes():
    HEART = [
        0b01010,
        0b11111,
        0b11111,
        0b11111,
        0b01110,
        0b00100,
        0b00000,
        0b00000,
    ]
    SMILE = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b10001,
        0b01010,
        0b00100,
        0b00000,
    ]
    load_chars({
        0: HEART,
        1: SMILE,
    })

def heart_eyes(seconds=5):
    load_heart_eyes()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1))
    sleep(seconds)


def thank_you_face(seconds=5):
    load_neutral_face()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 3)
    lcd.write_string("Thank you")
    sleep(seconds)

# =========================
# 11) 조는 표정
# =========================
def load_sleepy_face():
    EYE_HALF = [
        0b00000,
        0b11111,
        0b01110,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    ]
    EYE_CLOSED = [
        0b00000,
        0b00000,
        0b11111,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    ]
    MOUTH_SLEEPY = [
        0b00000,
        0b00000,
        0b00000,
        0b00100,
        0b00000,
        0b00100,
        0b00000,
        0b00000,
    ]

    load_chars({
        0: EYE_HALF,
        1: EYE_CLOSED,
        2: MOUTH_SLEEPY,
    })

def draw_sleepy(eye_mode=0):
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(eye_mode))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(eye_mode))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(2))
    lcd.cursor_pos = (1, 12)
    lcd.write_string("z")

def sleepy_demo(seconds=5):
    load_sleepy_face()
    elapsed = 0.0
    while elapsed < seconds:
        draw_sleepy(0)   # 반쯤 감김
        sleep(0.7)
        elapsed += 0.7
        if elapsed >= seconds:
            break

        draw_sleepy(1)   # 완전히 감김
        sleep(0.7)
        elapsed += 0.7

# =========================
# 12) 행복한 표정
# =========================
def load_happy_face():
    EYE_HAPPY = [
        0b00000,
        0b10001,
        0b01010,
        0b00100,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    ]
    MOUTH_SMILE_L = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b10000,
        0b01000,
        0b00110,
        0b00001,
    ]
    MOUTH_SMILE_R = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00001,
        0b00010,
        0b01100,
        0b10000,
    ]

    load_chars({
        0: EYE_HAPPY,
        1: MOUTH_SMILE_L,
        2: MOUTH_SMILE_R,
    })

def show_happy(seconds=5):
    load_happy_face()
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(0))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(1) + chr(2))
    sleep(seconds)

# =========================
# 13) 반가운 표정
# =========================
def load_greeting_face():
    EYE_OPEN = [
        0b01110,
        0b10001,
        0b10101,
        0b10001,
        0b01110,
        0b00000,
        0b00000,
        0b00000,
    ]
    EYE_HAPPY = [
        0b00000,
        0b10001,
        0b01010,
        0b00100,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    ]
    MOUTH_SMILE_L = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b10000,
        0b01000,
        0b00110,
        0b00001,
    ]
    MOUTH_SMILE_R = [
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00001,
        0b00010,
        0b01100,
        0b10000,
    ]

    load_chars({
        0: EYE_OPEN,
        1: EYE_HAPPY,
        2: MOUTH_SMILE_L,
        3: MOUTH_SMILE_R,
    })

def draw_greeting(happy=False):
    eye = 1 if happy else 0
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string(chr(eye))
    lcd.cursor_pos = (0, 11)
    lcd.write_string(chr(eye))
    lcd.cursor_pos = (1, 7)
    lcd.write_string(chr(2) + chr(3))

def greeting_demo(seconds=5):
    load_greeting_face()
    elapsed = 0.0
    while elapsed < seconds:
        draw_greeting(False)
        sleep(0.35)
        elapsed += 0.35
        if elapsed >= seconds:
            break

        draw_greeting(True)
        sleep(0.35)
        elapsed += 0.35

# =========================
# 14) MONEY 표정
# =========================
def money_face_eyes(seconds=5):
    lcd.clear()
    lcd.cursor_pos = (0, 4)
    lcd.write_string("$")
    lcd.cursor_pos = (0, 11)
    lcd.write_string("$")
    lcd.cursor_pos = (1, 5)
    lcd.write_string("MONEY!")
    sleep(seconds)

# =========================
# Controller Node
# =========================
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.face_sub = self.create_subscription(String, '/face_cmd', self.face_callback, qos)
        self.buzzer_sub = self.create_subscription(String, '/buzzer_cmd', self.buzzer_callback, qos)
        self.tail_sub = self.create_subscription(String, '/tail_cmd', self.tail_callback, qos)

        self.lcd_lock = threading.Lock()
        self.buzzer_lock = threading.Lock()
        self.tail_lock = threading.Lock()

        # 서보 2개
        self.tail_up = AngularServo(
            12,                         # 위쪽 서보
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        self.tail_down = AngularServo(
            13,                         # 아래쪽 서보
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        self.buzzer = buzzer

        lcd.clear()
        lcd.write_string("Face LCD Ready")
        booting_screen()
        self.get_logger().info('controller node started')

    # -------------------------
    # FACE
    # -------------------------
    def face_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Face cmd: {cmd}')
        threading.Thread(target=self.run_face, args=(cmd,), daemon=True).start()

    def run_face(self, cmd: str):
     with self.lcd_lock:
        if cmd == 'blink':
            show_blink(5)
        elif cmd == 'angry':
            show_angry(5)
        elif cmd == 'neutral':
            show_neutral(5)
        elif cmd == 'cry':
            show_cry(5)
        elif cmd == 'message':
            show_message(5)
        elif cmd == 'heart':
            show_heart(5)
        elif cmd == 'hearteye':
            heart_eyes(5)
        elif cmd == 'surprise':
            show_surprised(5)
        elif cmd == 'suspicious':
            show_suspicious(5)
        elif cmd == 'veyes':
            v_eyes(5)
        elif cmd == 'thankyou':
            thank_you_face(5)
        elif cmd == 'sleepy':
            sleepy_demo(5)
        elif cmd == 'happy':
            show_happy(5)
        elif cmd == 'greeting':
            greeting_demo(5)
        elif cmd == 'money':
            money_face_eyes(5)


        else:
            self.get_logger().warning(f'Unknown face command: {cmd}')
            lcd.clear()
            lcd.write_string("Unknown cmd")
            lcd.cursor_pos = (1, 0)
            lcd.write_string(cmd[:16])

    # -------------------------
    # BUZZER
    # -------------------------
    def buzzer_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Buzzer cmd: {cmd}')
        threading.Thread(target=self.play_buzzer_pattern, args=(cmd,), daemon=True).start()

    def play_note(self, note: str, duration: float):
        self.buzzer.play(Tone(note))
        sleep(duration)
        self.buzzer.stop()
        sleep(0.08)

    def play_buzzer_pattern(self, cmd: str):
     with self.buzzer_lock:
        if cmd == 'happy':
            # 도레미파 솔라시도
            melody = [
                ('C4', 0.25),
                ('D4', 0.25),
                ('E4', 0.25),
                ('F4', 0.25),
                ('G4', 0.25),
                ('A4', 0.25),
                ('B4', 0.25),
                ('C5', 0.40),
            ]
            for note, dur in melody:
                self.play_note(note, dur)

        elif cmd == 'danger':
            # 솔~미
            self.buzzer.play(Tone('G4'))
            sleep(0.7)
            self.buzzer.play(Tone('E4'))
            sleep(0.7)
            self.buzzer.stop()

        elif cmd == 'warning':
            # 삐삐
            for _ in range(2):
                self.buzzer.play(Tone('A5'))
                sleep(0.18)
                self.buzzer.stop()
                sleep(0.12)

        else:
            self.get_logger().warning(f'Unknown buzzer command: {cmd}')
            self.buzzer.stop()

    # -------------------------
    # TAIL
    # -------------------------
    def tail_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Tail cmd: {cmd}')
        threading.Thread(target=self.run_tail, args=(cmd,), daemon=True).start()

    def run_tail(self, cmd: str):
     with self.tail_lock:
        if cmd == 'normal':
            self.tail_basic()

        elif cmd == 'angry':
            self.tail_angry()

        elif cmd == 'friendly':
            self.tail_friendly()

        elif cmd == 'stop':
            self.tail_stop()

        else:
            self.get_logger().warning(f'Unknown tail command: {cmd}')

    def tail_stop(self):
        self.tail_up.angle = 10
        self.tail_down.angle = 0

    def tail_basic(self):
        # 평상시: 살랑살랑 크게
        for _ in range(3):
            self.tail_up.angle = 20
            self.tail_down.angle = -20
            sleep(0.35)

            self.tail_up.angle = 55
            self.tail_down.angle = 20
            sleep(0.35)

            self.tail_up.angle = 20
            self.tail_down.angle = -20
            sleep(0.35)

            self.tail_up.angle = -15
            self.tail_down.angle = -40
            sleep(0.35)

        self.tail_stop()

    def tail_angry(self):
        # 화날때: 바르르 떨기
        self.tail_up.angle = 10
        self.tail_down.angle = 0
        sleep(0.15)

        for _ in range(12):
            self.tail_up.angle = 20
            self.tail_down.angle = 15
            sleep(0.06)

            self.tail_up.angle = 10
            self.tail_down.angle = 0
            sleep(0.06)

        self.tail_stop()

    def tail_friendly(self):
        # 친해지자: 위쪽만 꺾이며 움직임
        self.tail_down.angle = 10

        for _ in range(4):
            self.tail_up.angle = 20
            sleep(0.25)
            self.tail_up.angle = 70
            sleep(0.25)

        self.tail_stop()

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        lcd.clear()
        buzzer.stop()
        node.tail_up.close()
        node.tail_down.close()
        buzzer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
