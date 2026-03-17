import cv2
import numpy as np

def nothing(x): pass

# 트랙바 윈도우 생성
cv2.namedWindow('Trackbars')
cv2.createTrackbar('L-H', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('L-S', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('L-V', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('U-H', 'Trackbars', 179, 179, nothing)
cv2.createTrackbar('U-S', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('U-V', 'Trackbars', 255, 255, nothing)

cap = cv2.VideoCapture(0) # 웹캠 연결

while True:
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 트랙바 값 읽기
    lh = cv2.getTrackbarPos('L-H', 'Trackbars')
    ls = cv2.getTrackbarPos('L-S', 'Trackbars')
    lv = cv2.getTrackbarPos('L-V', 'Trackbars')
    uh = cv2.getTrackbarPos('U-H', 'Trackbars')
    us = cv2.getTrackbarPos('U-S', 'Trackbars')
    uv = cv2.getTrackbarPos('U-V', 'Trackbars')

    lower = np.array([lh, ls, lv])
    upper = np.array([uh, us, uv])

    # 해당 범위만 흰색으로 보여주는 마스크
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask) # 찾는 물체만 하얗게 나와야 함
    cv2.imshow("Result", result)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()