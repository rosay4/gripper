import random


class Sampler:
    def __init__(self, sample_rate: float):
        """
        初始化采样器
        
        :param sample_rate: 浮点型采样率，范围应在0.0到1.0之间
        """
        if not 0.0 <= sample_rate <= 1.0:
            raise ValueError("采样率必须在0.0到1.0之间")
        self.sample_rate = sample_rate

    def should_sample(self) -> bool:
        """
        判断当前调用是否应该采样
        
        :return: 布尔值，True表示应该采样，False表示不应该采样
        """
        return random.random() < self.sample_rate