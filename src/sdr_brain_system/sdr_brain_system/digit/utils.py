# /home/ubuntu/kitty_burger_ws/kitty_burger/src/sdr_brain_system/sdr_brain_system/utils/digit_utils.py
import cv2
import numpy as np
from tensorflow.keras import layers, models

def preprocess_digit(roi_img):
    # 1. 테두리 제거
    offset = 5
    roi_clean = roi_img[offset:-offset, offset:-offset]
    
    # 2. 그레이스케일 및 반전
    gray = cv2.cvtColor(roi_clean, cv2.COLOR_BGR2GRAY)
    # gray = np.flip(gray, 1) # 카메라 방향에 따라 필요시 유지

    # 3. 대비 향상 및 블러
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    blur = cv2.GaussianBlur(enhanced, (5, 5), 1)

    # 4. 이진화 (C값을 7로 낮춰서 9의 동그라미가 안 깨지게 함)
    binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 21, 7)

    # 5. 모폴로지 (MORPH_CLOSE로 끊긴 선 연결)
    kernel = np.ones((3, 3), np.uint8)
    processed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
    
    return binary, processed # 디버그용으로 두 개 반환

def extract_digits(processed_img):
    contours, _ = cv2.findContours(processed_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    digit_rects = []
    roi_h, roi_w = processed_img.shape
    
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        area = cv2.contourArea(cnt)
        if area < 300 or area > (roi_h * roi_w * 0.95): continue
        if w / float(h) > 3.0: continue
        digit_rects.append((x, y, w, h))

    if not digit_rects: return None
    
    # 가장 큰 영역 선택
    x, y, w, h = sorted(digit_rects, key=lambda r: r[2]*r[3], reverse=True)[0]
    padding = 10
    x1, y1 = max(0, x-padding), max(0, y-padding)
    x2, y2 = min(roi_w, x+w+padding), min(roi_h, y+h+padding)
    digit_crop = processed_img[y1:y2, x1:x2]
    
    if digit_crop.size == 0: return None
    return cv2.resize(digit_crop, (28, 28), interpolation=cv2.INTER_AREA)

# --- 새로 추가되는 AI 클래스 ---
class DigitClassifier:
    def __init__(self, weights_path):
        self.model = self._build_model()
        try:
            self.model.load_weights(weights_path)
            print(f"✅ AI 가중치 로드 성공: {weights_path}")
        except Exception as e:
            print(f"❌ 가중치 로드 실패: {e}")

    def _build_model(self):
        return models.Sequential([
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

    def predict(self, digit_img):
        """28x28 이미지를 받아 숫자와 신뢰도를 반환"""
        if digit_img is None:
            return None, 0.0
        
        input_data = (digit_img / 255.0).reshape(1, 28, 28, 1).astype('float32')
        pred = self.model(input_data, training=False).numpy()
        digit = np.argmax(pred)
        conf = np.max(pred)
        return digit, conf