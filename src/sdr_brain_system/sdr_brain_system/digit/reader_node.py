# sdr_brain_system/digit/reader_node.py
import rclpy
import cv2
import numpy as np
import os
import threading
import queue
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

# 분리한 모듈 임포트 (패키지 경로 주의!)
from sdr_brain_system.digit.classifier import DigitClassifier
from sdr_brain_system.digit.utils import preprocess_digit, extract_digits

class DigitReaderNode(Node):
    def __init__(self):
        super().__init__('digit_reader_node')

        # 1. AI 모듈 초기화
        pkg_path = get_package_share_directory("sdr_brain_system")
        weights_path = os.path.join(pkg_path, "models", "model.weights.h5")
        self.classifier = DigitClassifier(weights_path)
        
        # 2. 통신 및 큐 설정
        self.img_queue = queue.Queue(maxsize=1)
        self.digit_pub = self.create_publisher(String, '/person/digit', 10)
        self.proc_pub = self.create_publisher(CompressedImage, '/vision/digit_proc/compressed', 10)

        # 3. 구독 설정 (상태에 따른 제어 포함)
        self.current_state = "ACT0_SLEEPY"
        self.create_subscription(String, '/mission_state', self.state_cb, 10)
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)

        # 4. 추론 쓰레드 가동
        threading.Thread(target=self.inference_worker, daemon=True).start()
        self.get_logger().info("🔢 숫자 인식 노드(모듈화 버전) 가동 시작")

    def state_cb(self, msg):
        self.current_state = msg.data.strip().upper()

    def image_callback(self, msg):
        # 배달 단계에서만 이미지를 큐에 넣음
        if self.current_state == "ACT4_DELIVERY" and self.img_queue.empty():
            self.img_queue.put(msg)
        
    def inference_worker(self):
        while rclpy.ok():
            try:
                msg = self.img_queue.get(timeout=1.0)
                frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue

                # ROI 설정 (중앙 150x150)
                h, w, _ = frame.shape
                size = 150
                roi = frame[(h-size)//2:(h+size)//2, (w-size)//2:(w+size)//2]

                # 1. 전처리 (digit_utils 사용)
                _, processed = preprocess_digit(roi)
                digit_img = extract_digits(processed)

                # 2. AI 예측 (classifier 사용)
                digit, conf = self.classifier.predict(digit_img)

                # 3. 결과 발행
                if digit is not None and conf > 0.7:
                    self.digit_pub.publish(String(data=str(digit)))
                    self.get_logger().info(f"🎯 인식 완료: {digit} ({conf:.2f})")

                # 4. 디버그 영상 송출
                proc_msg = CompressedImage()
                proc_msg.format = "jpeg"
                proc_msg.data = cv2.imencode('.jpg', processed, [cv2.IMWRITE_JPEG_QUALITY, 30])[1].tobytes()
                self.proc_pub.publish(proc_msg)

            except queue.Empty: continue
            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DigitReaderNode())
    rclpy.shutdown()