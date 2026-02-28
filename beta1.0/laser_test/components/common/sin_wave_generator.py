import math


class SinWaveGenerator:
    def __init__(self, period, amplitude, phase, center):
        self.period = period  # 周期，单位秒
        self.amplitude = amplitude  # 振幅，单位弧度
        self.phase = phase  # 相位，单位弧度
        self.center = center  # 中心偏移量
        self.time = 0.0  # 当前时间，单位秒

    def set_period(self, period):
        self.period = period

    def set_amplitude(self, amplitude):
        self.amplitude = amplitude

    def set_phase(self, phase):
        self.phase = phase

    def get_next_value(self, time_step):
        """计算下一个采样点的值
        Args:
            time_step: 时间步进，单位秒
        """
        self.time += time_step
        rad = 2 * math.pi * self.time / self.period + self.phase
        value = self.amplitude * math.sin(rad) + self.center
        # print(f"rad: {rad}, time: {self.time} value: {math.degrees(value)}")
        return value
