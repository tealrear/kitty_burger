import cv2
import numpy as np
import os

# 기존 모듈 임포트
from classifier import DigitClassifier
from utils import preprocess_digit, extract_digits
import sys
from ament_index_python.packages import get_package_share_directory

# [핵심] 현재 파일의 위치를 경로에 추가해서 utils를 찾을 수 있게 함
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

def main():
    # 1. 가중치 파일 경로 설정 (본인의 경로에 맞게 수정하세요)
    # 패키지 안에 있다면 절대 경로 혹은 상대 경로를 지정해야 합니다.

    # find ~/kitty_burger_ws -name model.weights.h5
    weights_path = "/home/ubuntu/kitty_burger_ws/install/sdr_brain_system/share/sdr_brain_system/models/model.weights.h5"
    
    if not os.path.exists(weights_path):
        print(f"❌ 가중치 파일을 찾을 수 없습니다: {weights_path}")
        return

    # 2. AI 모델 초기화
    classifier = DigitClassifier(weights_path)
    print("✅ AI 모델 로드 완료. 웹캠을 시작합니다.")

    # 3. 웹캠 열기 (0번 또는 1번)
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 4. ROI 설정 (reader_node.py와 동일하게 중앙 150x150)
        h, w, _ = frame.shape
        size = 150
        x1, y1 = (w - size) // 2, (h - size) // 2
        x2, y2 = x1 + size, y1 + size
        
        roi = frame[y1:y2, x1:x2]

        # 5. 전처리 및 숫자 추출 (utils.py 로직 사용)
        _, processed = preprocess_digit(roi)
        digit_img = extract_digits(processed)

        # 6. AI 예측
        digit, conf = classifier.predict(digit_img)

        # 7. 시각화 (화면에 결과 그리기)
        # 중앙 ROI 박스 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if digit is not None and conf > 0.7:
            text = f"Digit: {digit} ({conf:.2f})"
            cv2.putText(frame, text, (x1, y1 - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            print(f"🎯 인식: {digit} ({conf:.2f})")

        # 8. 화면 표시
        cv2.imshow("Digit Recognition Test", frame)
        cv2.imshow("Processed (AI Input)", processed) # 전처리된 결과도 함께 확인

        # 'q' 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()