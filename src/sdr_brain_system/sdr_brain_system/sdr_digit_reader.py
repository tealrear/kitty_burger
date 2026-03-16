import rclpy
import cv2
import numpy as np
import os
import threading
import queue # 큐 추가
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from tensorflow.keras import layers, models
from ament_index_python.packages import get_package_share_directory
import time

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
            pkg_path = get_package_share_directory("sdr_brain_system")
            weights_path = os.path.join(pkg_path, "models", "model.weights.h5")
            self.model.load_weights(weights_path)
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

    # [노트북 로직 100% 이식] 전처리 함수
    def preprocess_digit(self, roi_img):
        # 1. 가장자리 제거 (Border 제거)
        offset = 5
        roi_clean = roi_img[offset:-offset, offset:-offset]

        # 2. 그레이스케일 변환
        gray = cv2.cvtColor(roi_clean, cv2.COLOR_BGR2GRAY)

        # 3. 이미지 반전 (중요: 카메라 환경에 따라 다름. 노트북 주석 참고)
        # 만약 글자가 거꾸로 보인다면 아래 줄을 주석 해제하세요.
        # gray = np.flip(gray, 1)

        # 4. 대비 향상 (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)

        # 5. 잡음 제거 (Gaussian Blur)
        blur = cv2.GaussianBlur(enhanced, (5, 5), 3)

        # 6. 적응형 이진화 (Adaptive Threshold) - INV를 써서 배경을 검정으로
        binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 21, 25)

        # 7. 모폴로지 (Morphology Close) - 끊긴 글자 연결
        kernel = np.ones((3, 3), np.uint8)
        processed = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        
        return processed

    # 노트북의 추출 로직 이식
    def extract_digits(self, processed_img):
        contours, _ = cv2.findContours(processed_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        digit_rects = []
        roi_h, roi_w = processed_img.shape
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = cv2.contourArea(cnt)
            if area < 300 or area > (roi_h * roi_w * 0.95): continue
            if w / float(h) > 3.0: continue
            digit_rects.append((x, y, w, h))

        if not digit_rects: return []
        
        # 가장 큰 영역 선택
        x, y, w, h = sorted(digit_rects, key=lambda r: r[2]*r[3], reverse=True)[0]
        padding = 10
        x1, y1 = max(0, x-padding), max(0, y-padding)
        x2, y2 = min(roi_w, x+w+padding), min(roi_h, y+h+padding)
        digit_crop = processed_img[y1:y2, x1:x2]
        
        if digit_crop.size == 0: return []
        return [cv2.resize(digit_crop, (28, 28), interpolation=cv2.INTER_AREA)]

    def image_callback(self, msg):
        # ACT4 단계가 아니면 아무것도 하지 않음 (초록 사각형도 안 그려짐)
        if self.current_mission_state.strip().upper() != "ACT4_DELIVERY":
            return
        
        # 콜백에서는 큐에 넣기만 하고 즉시 종료 (막힘 방지)
        if self.img_queue.empty():
            self.img_queue.put(msg)
            print("Message put into Queue!")
        
    def inference_worker(self):
        print("!!! inference_worker loop start !!!")
        while rclpy.ok():
            try:
                msg = self.img_queue.get(timeout=1.0)
                if self.model is None: continue

                # 처리 시작 시간 측정
                start_time = time.time()

                np_arr = np.frombuffer(msg.data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if frame is None: continue

                h, w, _ = frame.shape
                size = 150
                x1, y1 = (w-size)//2, (h-size)//2
                roi = frame[y1:y1+size, x1:x1+size]

                processed = self.preprocess_digit(roi)
                digit_imgs = self.extract_digits(processed)

                # [최적화] AI 추론 부분
                if digit_imgs:
                    for img in digit_imgs:
                        # 정규화 및 텐서 변환
                        input_data = (img / 255.0).reshape(1, 28, 28, 1).astype('float32')
                        
                        # Keras 직접 호출 (predict보다 2~5배 빠름)
                        pred = self.model(input_data, training=False).numpy()
                        
                        digit = np.argmax(pred)
                        conf = np.max(pred)
                        
                        if conf > 0.7:
                            self.digit_pub.publish(String(data=str(digit)))
                            self.get_logger().info(f"✅ 인식: {digit} ({conf:.2f})")

                # [디버그 영상] 모든 프레임이 아닌 추론 때만 전송해서 부하 감소
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