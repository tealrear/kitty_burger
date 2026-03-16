import rclpy
import cv2
import numpy as np
import os
import threading
import queue
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tensorflow.keras import layers, models
from ament_index_python.packages import get_package_share_directory
from .utils.digit_utils import preprocess_digit, extract_digits, DigitClassifier

# [수정된 부분] utils 폴더에서 함수 가져오기
from .utils.digit_utils import preprocess_digit, extract_digits

class SdrDigitReaderNode(Node):
    def __init__(self):
        super().__init__('sdr_digit_reader')

        # 1. 에러 로그에 나온 구조 그대로 뼈대 만들기
        self.model = models.Sequential([
            layers.Input(shape=(28, 28, 1)),
            layers.Conv2D(32, kernel_size=(3, 3), padding='same', activation='relu'),
            layers.BatchNormalization(),
            layers.Conv2D(32, kernel_size=(3, 3), padding='same', activation='relu'),
            layers.MaxPooling2D(pool_size=(2, 2)),
            layers.Conv2D(64, kernel_size=(3, 3), padding='same', activation='relu'),
            layers.BatchNormalization(),
            layers.MaxPooling2D(pool_size=(2, 2)),
            layers.Flatten(),
            layers.Dense(256, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(10, activation='softmax')
        ])
        
        # 2. 추출한 가중치(Weights)만 입히기
        try:
            # 가중치 경로 설정 및 AI 클래스 초기화
            pkg_path = get_package_share_directory("sdr_brain_system")
            weights_path = os.path.join(pkg_path, "models", "model.weights.h5")
            self.classifier = DigitClassifier(weights_path)
            self.get_logger().info("✅ 가중치 로드 성공! 이제 동작합니다.")
        except Exception as e:
            self.get_logger().error(f"❌ 가중치 로드 실패: {e}")
        
        # 큐와 쓰레드 설정
        self.img_queue = queue.Queue(maxsize=1)
        self.digit_pub = self.create_publisher(String, '/person/digit', 10)

        # [디버그] 원본+박스 영상 / [디버그2] AI가 실제 보는 전처리(이진화) 영상
        self.debug_pub = self.create_publisher(CompressedImage, '/vision/digit_debug/compressed', 10)
        self.proc_pub = self.create_publisher(CompressedImage, '/vision/digit_proc/compressed', 10)

        self.sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        self.current_mission_state = "ACT0_SLEEPY"
        self.create_subscription(String, '/mission_state', self.state_cb, 10)

        # 추론 쓰레드 시작
        threading.Thread(target=self.inference_worker, daemon=True).start()
        self.get_logger().info("🔢 쓰레드 기반 숫자 인식 노드 가동")

    def state_cb(self, msg):
        self.current_mission_state = msg.data.strip().upper() # 공백 제거
        print(f"--- State Changed: [{self.current_mission_state}] ---")

    def image_callback(self, msg):
        if self.current_mission_state == "ACT4_DELIVERY" and self.img_queue.empty():
            self.img_queue.put(msg)
        
    def inference_worker(self):
        print("!!! inference_worker loop start !!!")
        while rclpy.ok():
            try:
                msg = self.img_queue.get(timeout=1.0)
                if self.model is None: continue

                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None: continue

                h, w, _ = frame.shape
                size = 150
                x1, y1 = (w-size)//2, (h-size)//2
                cv2.rectangle(frame, (x1, y1), (x1+size, y1+size), (0, 255, 0), 2)

                roi = frame[y1:y1+size, x1:x1+size]

                binary_raw, processed = preprocess_digit(roi)
                digit_img = extract_digits(processed)

                # 모듈화된 인터페이스 사용
                digit, conf = self.classifier.predict(digit_img)

                if digit is not None and conf > 0.7:
                    self.digit_pub.publish(String(data=str(digit)))
                    self.get_logger().info(f"✅ 인식: {digit} ({conf:.2f})")

                # 디버그 영상 전송
                proc_msg = CompressedImage()
                proc_msg.format = "jpeg"
                proc_msg.data = cv2.imencode('.jpg', processed, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
                self.proc_pub.publish(proc_msg)
                
                # 연산 속도 확인용 로그
                # self.get_logger().info(f"Inference Time: {time.time() - start_time:.4f}s")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Inference Error: {e}")

def main():
    rclpy.init()
    rclpy.spin(SdrDigitReaderNode())
    rclpy.shutdown()