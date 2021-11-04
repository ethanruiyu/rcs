import threading


class AtomicCounter:
    def __init__(self, initial=0, max_value=99999) -> None:
        self.value = initial
        self._lock = threading.Lock()
        self.max_value = max_value

    def increment(self, num=1):
        if self.value >= self.max_value:
            self.value = 0
        with self._lock:
            self.value += num
            return self.value