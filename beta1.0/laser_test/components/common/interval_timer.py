import time
from components.common.logger_wrapper import log_message


class IntervalTimer:
    def __init__(self, name: str, interval: float):
        """
        初始化定时器

        :param interval: 运行间隔（秒）
        """
        self.name = name
        self.interval = interval
        self.last_time = time.time()

    def is_ready(self) -> bool:
        """
        判断当前是否已经到达运行间隔

        :return: 如果到达运行间隔返回True，否则返回False
        """
        now = time.time()
        if now - self.last_time >= self.interval:
            self.last_time = now
            return True

        return False

    def get_remaining_time(self) -> float:
        """
        获取剩余时间直到下次触发

        :return: 剩余时间（秒），如果为负数表示已经超时
        """
        return self.interval - (time.time() - self.last_time)

    def change_interval(self, new_interval: float):
        """
        更改定时器的运行间隔

        :param new_interval: 新的运行间隔（秒）
        """
        self.interval = new_interval

    @staticmethod
    def calculate_min_sleep_time(timers: list["IntervalTimer"]) -> float:
        """
        计算多个定时器中的最小剩余时间

        :param timers: 定时器列表
        :return: 最小剩余时间（秒）
        """
        if not timers:
            return 0.001  # 默认1ms

        remaining_times = [timer.get_remaining_time() for timer in timers]
        min_remaining = min(remaining_times)

        # 确保返回正值，最小为0.001ms
        return max(min_remaining, 0.000001)

    @staticmethod
    def calculate_optimized_sleep_time(
        timers: list["IntervalTimer"], coefficient: float = 0.5
    ) -> float:
        """
        计算优化的休眠时间（最小间隔乘以系数）

        :param timers: 定时器列表
        :param coefficient: 系数（0-1之间）
        :return: 优化的休眠时间（秒）
        """
        min_time = IntervalTimer.calculate_min_sleep_time(timers)
        return max(min_time * coefficient, 0.000001)  # 最小1μs


# 示例用法：
if __name__ == "__main__":
    # 创建两个定时器
    control_timer = IntervalTimer("control_timer", 0.01)  # 10ms
    observe_timer = IntervalTimer("observe_timer", 0.004)  # 4ms

    timers = [control_timer, observe_timer]
    sleep_time = IntervalTimer.calculate_optimized_sleep_time(timers, 0.5)
    log_message(f"优化后的休眠时间: {sleep_time} 秒")

    log_message("定时器启动...")

    while True:
        current_time = time.time()

        for t in timers:
            if t.is_ready():
                log_message(f"{t.name} 任务在 {current_time} 执行")

        # 计算优化的休眠时间（最小间隔的0.5倍）
        time.sleep(min(sleep_time, 0.001))  # 最大休眠1ms
