import time


class StopWatch():
    def __init__(self):
        self.tic()

    def tic(self):
        """計測を開始/リセットする"""
        self.start_time = time.perf_counter()

    def check_elapsed(self, time_sec: float, reset: bool = True) -> bool:
        """指定した時間（秒）が経過したか確認する"""
        if self.tac() < time_sec:
            return False
        if reset:
            self.reset()
        return True

    def reset(self):
        """計測時間をリセットする"""
        self.tic()

    def tac(self, reset: bool = False) -> float:
        """経過時間（秒）を取得する"""
        elapsed = time.perf_counter() - self.start_time
        if reset:
            self.reset()
        return elapsed
