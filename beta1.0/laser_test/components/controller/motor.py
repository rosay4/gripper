from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class Motors:

    class MotorMode:
        POSITION = "position"
        VELOCITY = "velocity"
        TORQUE = "torque"
        MANUAL_DRAG = "manual_drag"

    class StateInfo:
        POSITION = "position"
        VELOCITY = "velocity"
        TORQUE = "torque"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice, motor_count: int):
        self.name = name
        self.device = device
        self.motor_count = motor_count
        self.position: list[float] = []
        self.velocity: list[float] = []
        self.torque: list[float] = []
        self.temperature: list[float] = []
        self.motor_mode = self.MotorMode.POSITION  # 默认模式

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if state and self.name in state and state[self.name]["type"] == "motor":
            self.state = state[self.name]
            ret = True
            if self.StateInfo.POSITION in self.state:
                self.position = self.state[self.StateInfo.POSITION]
            else:
                ret = False
            if self.StateInfo.VELOCITY in self.state:
                self.velocity = self.state[self.StateInfo.VELOCITY]
            else:
                ret = False
            if self.StateInfo.TORQUE in self.state:
                self.torque = self.state[self.StateInfo.TORQUE]
            else:
                ret = False
            return ret
        return False

    def get_position(self) -> tuple[bool, list[float]]:
        """获取当前位置"""
        if self.get_state():
            return True, self.position

        return False, []

    def get_last_position(self) -> list[float]:
        """获取当前缓存的位置"""
        return self.position

    def set_positions(self, positions: list[float]) -> bool:
        """移动到指定位置"""
        if not positions or len(positions) != len(self.position):
            log_message(f"Error: Invalid positions {positions}")
            return False

        action = {self.name: {"type": self.motor_mode, self.motor_mode: positions}}
        self.device.set_action(action)
        return True

    def get_torque(self) -> tuple[bool, list[float]]:
        """获取当前力矩"""
        if self.get_state():
            return True, self.torque

        return False, []

    def get_last_torque(self) -> list[float]:
        """获取当前缓存的力矩"""
        return self.torque

    def set_torques(self, torques: list[float]) -> bool:
        """设置力矩"""
        if not torques or len(torques) != len(self.torque):
            log_message(f"Error: Invalid torques {torques}")
            return False

        action = {self.name: {"type": self.motor_mode, self.motor_mode: torques}}
        self.device.set_action(action)
        return True

    def get_velocity(self) -> tuple[bool, list[float]]:
        """获取当前速度"""
        if self.get_state():
            return True, self.velocity

        return False, []

    def get_last_velocity(self) -> list[float]:
        """获取当前缓存的速度"""
        return self.velocity

    def set_velocities(self, velocities: list[float]) -> bool:
        """设置速度"""
        if not velocities or len(velocities) != len(self.velocity):
            log_message(f"Error: Invalid velocities {velocities}")
            return False

        action = {self.name: {"type": self.motor_mode, self.motor_mode: velocities}}
        self.device.set_action(action)
        return True

    def get_temperature(self) -> tuple[bool, list[float]]:
        """获取当前温度"""
        ret = self.device.send_cmd(self.name, {"command": "get_temperature"})
        # wrapper_log_message(ret)
        if "value" in ret:
            self.temperature = ret["value"]
            return True, self.temperature
        return False, []

    def get_last_temperature(self) -> list[float]:
        """获取当前缓存温度"""
        return self.temperature

    def get_mode(self) -> str:
        """获取当前电机模式"""
        mode = self.device.send_cmd(self.name, {"command": "get_control_mode"})["mode"]
        return mode

    def set_mode(self, mode: str) -> bool:
        """设置电机模式"""
        if mode not in (
            self.MotorMode.POSITION,
            self.MotorMode.VELOCITY,
            self.MotorMode.TORQUE,
            self.MotorMode.MANUAL_DRAG,
        ):
            log_message(f"Error: Invalid motor mode {mode}")
            return False

        self.motor_mode = mode
        try:
            ret = self.device.send_cmd(
                self.name, {"command": "set_control_mode", "mode": mode}
            )
        except Exception as e:
            log_message(f"Error setting motor mode: {e}")
            return False
        # wrapper_log_message(ret)
        return True

    def set_id(self, id: int) -> bool:
        """设置电机ID"""
        return False  # 目前不支持

    def get_status(self) -> str:
        """获取电机状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查电机是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def reset_motors(self) -> bool:
        """重置电机"""
        ret = self.device.send_cmd(self.name, {"command": "reset_motor"})
        # wrapper_log_message(ret)
        return True

    def set_zero(self) -> bool:
        """设置当前位置为零点"""
        ret = self.device.send_cmd(self.name, {"command": "set_zero"})
        # wrapper_log_message(ret)
        return True

    def get_limit(self) -> tuple[bool, list[bool], list[float], list[float]]:
        """获取电机限制参数"""
        ret = self.device.send_cmd(self.name, {"command": "get_limit"})
        # wrapper_log_message(ret)
        if "enabled" in ret and "lower" in ret and "upper" in ret:
            return True, ret["enabled"], ret["lower"], ret["upper"]
        return False, {}

    def set_limit(self, index, enabled, lower, upper) -> bool:
        """获取电机限制参数"""
        ret, origin_enabled, origin_lower, origin_upper = self.get_limit()
        if not ret:
            log_message("获取电机限位失败")
            return False

        origin_enabled[index] = enabled
        origin_lower[index] = lower
        origin_upper[index] = upper
        cmd = {
                "command": "set_limit",
                "enabled": origin_enabled,
                "lower": origin_lower,
                "upper": origin_upper,
            }
        log_message(f"设置电机限位参数: {cmd}")
        ret = self.device.send_cmd(
            self.name,
            cmd,
        )
        if ret:
            return True
        return False


class EyouMotors(Motors):
    def __init__(self, name: str, device: LocalDevice | RemoteDevice, motor_count: int):
        super().__init__(name, device, motor_count)

    def set_pp_mode_param(self, acc: list[float], dec: list[float], vel: list[float]) -> bool:
        """设置电机位置模式参数"""
        cmd = {
            "command": "set_profile_position_mode_param",
            "max_acceleration": acc,
            "max_deceleration": dec,
            "max_velocity": vel,
        }
        log_message(f"设置电机位置模式参数: {cmd}")
        try:
            ret = self.device.send_cmd(
                self.name,
                cmd,
            )
        except Exception as e:
            log_message(f"Error setting motor pp mode param: {e}")
            return False

        if ret:
            return True
        return False
