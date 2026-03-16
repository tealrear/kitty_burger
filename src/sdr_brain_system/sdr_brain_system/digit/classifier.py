# sdr_brain_system/digit/classifier.py
import numpy as np
from tensorflow.keras import layers, models

class DigitClassifier:
    def __init__(self, weights_path):
        # 1. 모델 구조 정의 (기존과 동일)
        self.model = self._build_model()
        
        # 2. 가중치 로드
        try:
            self.model.load_weights(weights_path)
            print(f"✅ [AI] 가중치 로드 완료: {weights_path}")
        except Exception as e:
            print(f"❌ [AI] 가중치 로드 실패: {e}")

    def _build_model(self):
        return models.Sequential([
            layers.Input(shape=(28, 28, 1)),
            layers.Conv2D(32, (3, 3), padding='same', activation='relu'),
            layers.BatchNormalization(),
            layers.Conv2D(32, (3, 3), padding='same', activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Conv2D(64, (3, 3), padding='same', activation='relu'),
            layers.BatchNormalization(),
            layers.MaxPooling2D((2, 2)),
            layers.Flatten(),
            layers.Dense(256, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(10, activation='softmax')
        ])

    def predict(self, digit_img):
        """28x28 이미지를 받아 (숫자, 신뢰도) 반환"""
        if digit_img is None:
            return None, 0.0
            
        # 전처리: 정규화 및 차원 확장
        input_data = (digit_img / 255.0).reshape(1, 28, 28, 1).astype('float32')
        
        # 추론
        pred = self.model(input_data, training=False).numpy()
        digit = int(np.argmax(pred))
        conf = float(np.max(pred))
        
        return digit, conf