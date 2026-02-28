from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class Battery:

    class StateInfo:
        PERCENTAGE = "percentage"
        CURRENT = "current"
        VOLTAGE = "voltage"
        TEMPERATURE = "temperature"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.percentage: int = 0
        self.current: float = 0.0
        self.voltage: float = 0.0
        self.temperature: float = 0.0

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if state and self.name in state and state[self.name]["type"] == "battery":
            self.state = state[self.name]
            ret = False
            if self.StateInfo.PERCENTAGE in self.state:
                self.percentage = self.state[self.StateInfo.PERCENTAGE]
                ret = True
            if self.StateInfo.CURRENT in self.state:
                self.current = self.state[self.StateInfo.CURRENT]
                ret = True
            if self.StateInfo.VOLTAGE in self.state:
                self.voltage = self.state[self.StateInfo.VOLTAGE]
                ret = True
            if self.StateInfo.TEMPERATURE in self.state:
                self.temperature = self.state[self.StateInfo.TEMPERATURE]
                ret = True
            return ret
        return False

    def get_percentage(self) -> tuple[bool, int]:
        """获取当前电量百分比"""
        if self.get_state():
            return True, self.percentage

        return False, 0

    def get_last_percentage(self) -> int:
        """获取当前缓存的电量百分比"""
        return self.percentage

    def get_current(self) -> tuple[bool, float]:
        """获取当前电流"""
        if self.get_state():
            return True, self.current

        return False, 0.0

    def get_last_current(self) -> float:
        """获取当前缓存的电流"""
        return self.current

    def get_voltage(self) -> tuple[bool, float]:
        """获取当前电压"""
        if self.get_state():
            return True, self.voltage

        return False, 0.0

    def get_last_voltage(self) -> float:
        """获取当前缓存的电压"""
        return self.voltage

    def get_temperature(self) -> tuple[bool, float]:
        """获取当前温度"""
        if self.get_state():
            return True, self.temperature

        return False, 0.0

    def get_last_temperature(self) -> float:
        """获取当前缓存的温度"""
        return self.temperature

    def get_status(self) -> str:
        """获取电池状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查电池是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None
