import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    import tensorflow.lite as tflite

class MoneyDetectorNode(Node):
    def __init__(self):
        super().__init__('detect_money')
        
        # 1. 경로 설정
        pkg_path = get_package_share_directory('sdr_brain_system')
        model_path = os.path.join(pkg_path, 'models', 'model_unquant.tflite')
        label_path = os.path.join(pkg_path, 'models', 'labels.txt')
        
        # 2. TFLite 초기화
        self.interpreter = tflite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # 3. 라벨 로드 (에러 방지 처리 추가)
        try:
            with open(label_path, 'r') as f:
                self.labels = [line.strip().split(' ', 1)[1] for line in f.readlines() if ' ' in line]
            self.get_logger().info(f"✅ 로드된 라벨: {self.labels}")
        except Exception as e:
            self.get_logger().error(f"❌ 라벨 로드 실패: {e}")

        # 4. 통신 설정
        self.create_subscription(CompressedImage, '/image_raw/compressed', self.img_cb, 10)
        self.publisher = self.create_publisher(String, '/vision_fast_data', 10)
        
        self.count = 0 # 프레임 카운터

    def img_cb(self, msg):
        self.count += 1
        
        # [디버깅] 10프레임마다 이미지 수신 알림
        if self.count % 10 == 0:
            self.get_logger().info("📷 영상 수신 중...", throttle_duration_sec=2.0)

        # 이미지 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return
        
        # 전처리
        input_shape = self.input_details[0]['shape']
        img = cv2.resize(frame, (input_shape[1], input_shape[2]))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        input_data = np.expand_dims(img, axis=0).astype(np.float32)
        input_data = (input_data / 127.5) - 1.0

        # 추론
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        
        idx = np.argmax(output_data[0])
        label = self.labels[idx]
        conf = output_data[0][idx]

        # [디버깅] AI가 보고 있는 모든 결과 출력 (필터링 전)
        # 만약 아무것도 안 뜬다면 이 로그가 찍히는지 확인하세요.
        if self.count % 5 == 0:
             print(f"🔍 AI 분석 결과 -> [{label}]: {conf:.2f}")

        mapping = {
            "won1000": "MONEY_BLUE",
            "won5000": "MONEY_RED",
            "won10000": "MONEY_GREEN",
            "none": "NONE"
        }

        # 0.8이 너무 높을 수 있으니 테스트 시에는 0.6 정도로 낮춰보세요.
        if conf > 0.6 and label in mapping:
            result = mapping[label]
            if result != "NONE":
                self.publisher.publish(String(data=result))
                self.get_logger().info(f"💰 [발행함] {result} ({conf:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = MoneyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()