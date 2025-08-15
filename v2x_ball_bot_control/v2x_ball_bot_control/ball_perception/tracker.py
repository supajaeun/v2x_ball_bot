# tracker.py
from collections import deque
import math

class SimpleTracker:
    def __init__(self, maxlen=5):
        self.track_history = {}
        self.maxlen = maxlen

    def update(self, obj_id, x, y, t):
        if obj_id not in self.track_history:
            self.track_history[obj_id] = deque(maxlen=self.maxlen)
        self.track_history[obj_id].append((x, y, t))

        if len(self.track_history[obj_id]) >= 2:
            (x1, y1, t1), (x2, y2, t2) = self.track_history[obj_id][0], self.track_history[obj_id][-1]
            dt = t2 - t1
            dist = math.hypot(x2 - x1, y2 - y1)
            speed = dist / dt if dt > 0 else 0.0
            return speed, speed < 0.05
        return 0.0, False
