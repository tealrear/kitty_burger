import rclpy
import cv2
import json
import os
import threading
import queue
import numpy as np
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO
import mediapipe as mp
from ament_index_python.packages import get_package_share_directory

CONF_THRESHOLD = 0.4       # 얼굴/표정 인식 최소 신뢰도
MANAGER_THRESHOLD = 0.6    # manager로 인식할 최소 확률
LOG_INTERVAL = 2.0         # 로그 최소 간격 (초)

class DetectHumanNode(Node):
    def __init__(self):
        super().__init__('detect_human_node')

        pkg_path = get_package_share_directory("sdr_brain_system")
        self.host_model = YOLO(os.path.join(pkg_path, "models", "hostface.pt"))
        self.expression_model = YOLO(os.path.join(pkg_path, "models", "best.pt"))
        self.hands = mp.solutions.hands.Hands(max_num_hands=1)

        self.img_queue = queue.Queue(maxsize=1)
        self.current_state = "ACT0_SLEEPY"
        self.last_hand_log_time = 0.0
        self.last_face_log_time = 0.0

        self.label_map = {0: "anger", 1: "fear", 2: "happy", 3: "neutral", 4: "sad"}

        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.state_sub = self.create_subscription(String, '/mission_state', self.state_cb, 10)
        
        self.hand_pub = self.create_publisher(String, '/person/hand', 10)
        self.exp_pub = self.create_publisher(String, '/person/expression', 10)
        self.face_id_pub = self.create_publisher(String, '/person/face_id', 10)

        threading.Thread(target=self.inference_worker, daemon=True).start()
        self.get_logger().info("🚀 [AI] 인적 탐지 노드가 정상적으로 시작되었습니다.")

    def state_cb(self, msg):
        self.current_state = msg.data

    def image_callback(self, msg):
        # AI가 작동해야 하는 상태에서만 큐에 데이터 삽입
        if self.current_state in ["ACT3_AUTHENTICATE", "ACT5_PAYMENT"]:
            if self.img_queue.empty():
                self.img_queue.put(msg)

    # ---------------- 손가락 계산 ----------------
    def angle(self, a, b, c):
        ba = np.array([a.x - b.x, a.y - b.y])
        bc = np.array([c.x - b.x, c.y - b.y])
        dot = np.dot(ba, bc)
        norm_ba = np.linalg.norm(ba)
        norm_bc = np.linalg.norm(bc)
        if norm_ba * norm_bc == 0: return 0
        return math.degrees(math.acos(max(-1.0, min(1.0, dot / (norm_ba * norm_bc)))))

    def is_finger_straight(self, landmarks, tip_idx, pip_idx, mcp_idx):
        return self.angle(landmarks[tip_idx], landmarks[pip_idx], landmarks[mcp_idx]) > 160

    def get_gesture(self, hand_landmarks):
        l = hand_landmarks.landmark
        
        idx_s = self.is_finger_straight(l, 8, 6, 5)
        mid_s = self.is_finger_straight(l, 12, 10, 9)
        rng_s = self.is_finger_straight(l, 16, 14, 13)
        pnk_s = self.is_finger_straight(l, 20, 18, 17)

        thm_o = (l[4].x - l[3].x)**2 + (l[4].y - l[3].y)**2 > 0
        if idx_s and not mid_s and not rng_s and not pnk_s: return "지시"
        if idx_s and mid_s and not rng_s and not pnk_s: return "브이"
        if idx_s and mid_s and rng_s and pnk_s: return "보"
        if not idx_s and not mid_s and not rng_s and not pnk_s and thm_o: return "엄지척"
        return "not known"

    def detect_hand(self, image):
        if image is None or image.size == 0:
            return None
        res = self.hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        if res.multi_hand_landmarks:
            for hand_landmarks in res.multi_hand_landmarks:
                gesture = self.get_gesture(hand_landmarks)
                tip_x = hand_landmarks.landmark[8].x
                data = {"gesture": gesture, "tip_x": tip_x}
                self.hand_pub.publish(String(data=json.dumps(data)))
                # 2초 간격 로그
                if time.time() - self.last_hand_log_time >= LOG_INTERVAL:
                    self.get_logger().info(f"[HAND] Gesture: {gesture}, tip_x: {tip_x:.2f}")
                    self.last_hand_log_time = time.time()
        return res.multi_hand_landmarks

    # ---------------- 얼굴/표정 계산 ----------------
    def detect_expression(self, image):
        if image is None or image.size == 0: return None
        res = self.expression_model(image, verbose=False)
        for r in res:
            for b in r.boxes:
                conf = float(b.conf)
                if conf >= CONF_THRESHOLD:
                    label = self.label_map.get(int(b.cls), "unknown")
                    # [상태 기반 토픽 발행] 결제/교감 단계에서만 표정 발행
                    if self.current_state == "ACT5_PAYMENT":
                        self.exp_pub.publish(String(data=json.dumps({"expression": label})))
        return res

    # ---------------- 메인 추론 ----------------
    def inference_worker(self):
        while rclpy.ok():
            try:
                msg = self.img_queue.get(timeout=1.0)
                frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.size == 0:
                    self.get_logger().warn("Empty frame received, skipping...")
                    continue

                # ---------------- 얼굴/주인 인식 ----------------
                host_results = self.host_model(frame, verbose=False)
                face_boxes = []
                for r in host_results:
                    for box in r.boxes:
                        conf = float(box.conf)
                        label = self.host_model.names[int(box.cls)]

                        if label == "manager" and conf >= MANAGER_THRESHOLD:
                            # 주인임이 확인되면 face_id 토픽 발행
                            self.face_id_pub.publish(String(data="manager"))

                        if conf < CONF_THRESHOLD:
                            continue
                        coords = np.ravel(box.xyxy.cpu().numpy())
                        x1, y1, x2, y2 = [int(c) for c in coords]
                        # 안전하게 crop
                        x1, y1 = max(0, x1), max(0, y1)
                        x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
                        if x2 <= x1 or y2 <= y1:
                            continue
                        face_boxes.append((x1, y1, x2, y2, label, conf))

                # ---------------- 손 인식 ----------------
                hand_results = self.detect_hand(frame)
                hand_boxes = []
                if hand_results:
                    h, w, _ = frame.shape
                    for landmarks in hand_results:
                        xs = [l.x for l in landmarks.landmark]
                        ys = [l.y for l in landmarks.landmark]
                        x1, y1 = int(min(xs)*w), int(min(ys)*h)
                        x2, y2 = int(max(xs)*w), int(max(ys)*h)
                        # 안전 crop
                        x1, y1 = max(0, x1), max(0, y1)
                        x2, y2 = min(w, x2), min(h, y2)
                        if x2 <= x1 or y2 <= y1:
                            continue
                        hand_boxes.append((x1, y1, x2, y2))

                # ---------------- 얼굴/손 영역 중 큰 영역만 처리 ----------------
                candidate = []
                for x1, y1, x2, y2, label, conf in face_boxes:
                    area = (x2-x1)*(y2-y1)
                    candidate.append(('face', area, (x1, y1, x2, y2, label, conf)))
                for x1, y1, x2, y2 in hand_boxes:
                    area = (x2-x1)*(y2-y1)
                    candidate.append(('hand', area, (x1, y1, x2, y2)))

                if candidate:
                    candidate.sort(key=lambda x: x[1], reverse=True)
                    kind, _, data = candidate[0]

                    if kind == 'face':
                        x1, y1, x2, y2, label, conf = data
                        best_crop = frame[y1:y2, x1:x2]
                        if self.current_state == "ACT5_PAYMENT":
                            self.detect_expression(best_crop)
                        # manager threshold 적용
                        if label == "manager" and conf >= MANAGER_THRESHOLD:
                            self.get_logger().info(f"[MANAGER] Detected at [{x1},{y1},{x2},{y2}], conf={conf:.2f}")
                    elif kind == 'hand':
                        x1, y1, x2, y2 = data
                        hand_crop = frame[y1:y2, x1:x2]
                        self.detect_hand(hand_crop)

            except Exception as e:
                self.get_logger().error(f"Inference error: {e}")
                continue

def main(args=None):
    rclpy.init(args=args)
    node = DetectHumanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
