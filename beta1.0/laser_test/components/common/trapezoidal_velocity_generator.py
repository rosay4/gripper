import math


class TrapezoidalVelocityGenerator:
    def __init__(self, acceleration, deceleration, max_velocity, total_distance):
        """
        初始化梯形速度生成器

        Args:
            acceleration: 加速度 (units/s²)
            deceleration: 减速度 (units/s²)，应为正值
            max_velocity: 最大速度 (units/s)
            total_distance: 总运行距离 (units)
        """
        self.acceleration = acceleration
        self.deceleration = deceleration
        self.max_velocity = max_velocity
        self.total_distance = total_distance

        # 计算各阶段时间
        self.acceleration_time = max_velocity / acceleration  # 加速时间
        self.deceleration_time = max_velocity / deceleration  # 减速时间

        # 计算加速阶段距离
        acceleration_distance = 0.5 * self.acceleration * (self.acceleration_time**2)
        # 计算减速阶段距离
        deceleration_distance = 0.5 * self.deceleration * (self.deceleration_time**2)

        # 计算匀速阶段距离
        constant_velocity_distance = (
            self.total_distance - acceleration_distance - deceleration_distance
        )

        # 如果匀速距离为负，说明无法达到最大速度，需要调整
        if constant_velocity_distance < 0:
            # 重新计算，假设加速和减速阶段末速度相等
            # v_max^2/(2*a) + v_max^2/(2*d) = total_distance
            # v_max = sqrt(2 * total_distance * a * d / (a + d))
            self.max_velocity = math.sqrt(
                2
                * self.total_distance
                * self.acceleration
                * self.deceleration
                / (self.acceleration + self.deceleration)
            )

            self.acceleration_time = self.max_velocity / self.acceleration
            self.deceleration_time = self.max_velocity / self.deceleration

            # 此时匀速阶段距离为0，匀速时间为0
            self.constant_velocity_time = 0.0
        else:
            # 计算匀速时间
            self.constant_velocity_time = constant_velocity_distance / self.max_velocity

        # 计算总时间
        self.total_time = (
            self.acceleration_time
            + self.constant_velocity_time
            + self.deceleration_time
        )
        print(
            f"TrapezoidalVelocityGenerator initialized: total_time={self.total_time:.3f}s, max_velocity={self.max_velocity:.3f} units/s"
        )

    def get_velocity_and_status(self, elapsed_time):
        """
        根据运行时间计算当前速度以及是否已完成

        Args:
            elapsed_time: 运行时间 (seconds)

        Returns:
            tuple: (velocity, is_finished) 当前速度和是否完成
        """
        # 判断是否已完成
        is_finished = elapsed_time >= self.total_time

        if is_finished:
            # 运动结束，速度为0
            return 0.0, True

        # 计算当前阶段的速度
        if elapsed_time <= self.acceleration_time:
            # 加速阶段
            current_velocity = self.acceleration * elapsed_time
        elif elapsed_time <= self.acceleration_time + self.constant_velocity_time:
            # 匀速阶段
            current_velocity = self.max_velocity
        elif elapsed_time <= self.total_time:
            # 减速阶段
            deceleration_time = (
                elapsed_time - self.acceleration_time - self.constant_velocity_time
            )
            current_velocity = self.max_velocity - self.deceleration * deceleration_time
        else:
            # 运动结束，速度为0
            current_velocity = 0.0

        return current_velocity, is_finished

    def get_total_time(self):
        """获取运动总时间"""
        return self.total_time

    def get_position(self, elapsed_time):
        """
        根据运行时间计算当前位移

        Args:
            elapsed_time: 运行时间 (seconds)

        Returns:
            float: 当前位移
        """
        if elapsed_time <= self.acceleration_time:
            # 加速阶段位移
            position = 0.5 * self.acceleration * (elapsed_time**2)
        elif elapsed_time <= self.acceleration_time + self.constant_velocity_time:
            # 匀速阶段位移
            acceleration_distance = (
                0.5 * self.acceleration * (self.acceleration_time**2)
            )
            constant_velocity_time = elapsed_time - self.acceleration_time
            position = (
                acceleration_distance + self.max_velocity * constant_velocity_time
            )
        elif elapsed_time <= self.total_time:
            # 减速阶段位移
            acceleration_distance = (
                0.5 * self.acceleration * (self.acceleration_time**2)
            )
            constant_velocity_distance = self.max_velocity * self.constant_velocity_time
            deceleration_time = (
                elapsed_time - self.acceleration_time - self.constant_velocity_time
            )
            deceleration_distance = (
                self.max_velocity * deceleration_time
                - 0.5 * self.deceleration * (deceleration_time**2)
            )
            position = (
                acceleration_distance
                + constant_velocity_distance
                + deceleration_distance
            )
        else:
            # 运动结束，总位移为总距离
            position = self.total_distance

        return position

    def get_velocity_by_remaining_distance(self, remaining_distance):
        """
        根据剩余距离计算当前允许的最大速度

        Args:
            remaining_distance: 剩余距离 (units)

        Returns:
            float: 当前允许的最大速度
        """
        if remaining_distance < 0:
            return 0.0

        # 根据 v^2 = 2 * a * d 计算能够安全减速到0的最大速度
        # 这里 a 是减速度 deceleration, d 是 remaining_distance
        required_velocity = math.sqrt(2 * self.deceleration * remaining_distance)
        # print(
        #     f"剩余距离={remaining_distance:.3f}, 减速度={self.deceleration:.3f}, 输出速度={required_velocity:.3f}"
        # )

        # 实际速度不能超过设定的最大速度
        current_velocity = min(self.max_velocity, required_velocity)
        # print(f"实际速度={current_velocity:.3f}")

        return current_velocity
