# sdr_brain_system/common/filters.py
import time

class ValueFilter:
    def __init__(self):
        # 여러 종류의 데이터를 개별적으로 관리하기 위한 딕셔너리
        self.timers = {}

    def is_confirmed(self, category, new_val, threshold=1.5):
        """
        category: "digit", "color", "thumb" 등 구분자
        new_val: 현재 인식된 값
        threshold: 확정에 필요한 시간(초)
        """
        now = time.time()
        
        # 1. 카테고리가 처음 등장하면 초기화
        if category not in self.timers:
            self.timers[category] = {"val": None, "start": now}

        timer = self.timers[category]

        # 2. 값이 없거나 "none"이면 타이머 리셋 및 무효 처리
        if new_val in ["none", "NONE", None]:
            timer.update({"val": new_val, "start": now})
            return False

        # 3. 이전에 보던 값과 다른 값이 들어오면 타이머 리셋
        if new_val != timer["val"]:
            timer.update({"val": new_val, "start": now})
            return False

        # 4. 동일한 값이 계속 유지되고 있다면 시간 계산
        duration = now - timer["start"]
        return duration >= threshold