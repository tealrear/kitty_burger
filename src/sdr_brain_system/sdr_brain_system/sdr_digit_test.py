# /home/ubuntu/kitty_burger_ws/kitty_burger/src/sdr_brain_system/sdr_brain_system/sdr_digit_test.py

import sys
import os
import cv2
import numpy as np

# [핵심] 현재 파일의 위치를 경로에 추가해서 utils를 찾을 수 있게 함
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# 이제 . 없이 바로 import 합니다.
from utils.digit_utils import preprocess_digit, extract_digits

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("카메라를 열 수 없습니다!")
        return

    print("--- 숫자 인식 전처리 테스트 시작 (q: 종료) ---")
    
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        frame = cv2.flip(frame, 1) # 거울 모드
        h, w, _ = frame.shape
        size = 150
        x1, y1 = (w-size)//2, (h-size)//2
        roi = frame[y1:y1+size, x1:x1+size].copy()

        # utils/digit_utils.py의 함수 호출
        binary_raw, processed = preprocess_digit(roi)
        digit_img = extract_digits(processed)

        # 화면 출력
        cv2.rectangle(frame, (x1, y1), (x1+size, y1+size), (0, 255, 0), 2)
        cv2.imshow("1. Original", frame)
        cv2.imshow("2. Binary (Check 9 loop!)", binary_raw)
        cv2.imshow("3. Processed (Final)", processed)

        if digit_img is not None:
            # AI에게 들어가는 최종 이미지를 크게 보여줌
            cv2.imshow("4. AI Input (28x28)", cv2.resize(digit_img, (140, 140)))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()