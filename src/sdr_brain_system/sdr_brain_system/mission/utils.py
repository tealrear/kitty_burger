import time
from std_msgs.msg import String

class StateFilter:
    def __init__(self):
        self.timers = {}

    def is_confirmed(self, category, new_val, threshold=2.0):
        now = time.time()
        if category not in self.timers:
            self.timers[category] = {"val": None, "start": now}

        timer = self.timers[category]
        if new_val in ["none", "NONE", None] or new_val != timer["val"]:
            timer.update({"val": new_val, "start": now})
            return False
        
        return (now - timer["start"]) >= threshold

class RobotCommandManager:
    def __init__(self, node):
        self.node = node
        self.last_sent = {"face": "", "buzzer": "", "tail": ""}

    def send(self, face=None, buzzer=None, tail=None):
        for cmd_type, val in [("face", face), ("buzzer", buzzer), ("tail", tail)]:
            if val and val != self.last_sent[cmd_type]:
                pub = getattr(self.node, f"{cmd_type}_pub")
                pub.publish(String(data=val))
                self.last_sent[cmd_type] = val