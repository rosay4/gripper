from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class Tactile:

    class StateInfo:
        VALUE = "value"
        SCALE = "scale"
        MAX_VALUE = "max_value"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.last_state_time = {}
        self.force: list[float] = []
        self.scale: float = 1.0
        self.max_value: int = 0

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if (
            state
            and self.name in state
            and state[self.name]["type"] == "single_tactile_array"
        ):
            self.state = state[self.name]
            self.last_state_time = self.state["time"]
            ret = True
            if self.StateInfo.VALUE in self.state:
                self.force = self.state[self.StateInfo.VALUE]
            else:
                ret = False
            if self.StateInfo.SCALE in self.state:
                self.scale = self.state[self.StateInfo.SCALE]
            if self.StateInfo.MAX_VALUE in self.state:
                self.max_value = self.state[self.StateInfo.MAX_VALUE]
            return ret
        return False

    def get_last_state_time(self) -> dict:
        """获取上次状态更新时间"""
        return self.last_state_time

    def get_force(self) -> tuple[bool, list[float]]:
        """获取当前力"""
        if self.get_state():
            return True, self.force

        return False, []

    def get_last_force(self) -> list[float]:
        """获取当前缓存的力"""
        return self.force

    def get_status(self) -> str:
        """获取传感器状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查触觉传感器是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def reset_tactile(self) -> bool:
        """重置力矩传感器"""
        ret = self.device.send_cmd(self.name, {"command": "reset_sensor"})
        # wrapper_log_message(ret)
        return True

    def calibrate_zero(self, index: int) -> bool:
        """校准力矩传感器零点"""
        ret = self.device.send_cmd(
            self.name, {"command": "calibrate_zero", "index": index}
        )
        # wrapper_log_message(ret)
        return True

    def calibrate_force(self, index: int, force: float) -> bool:
        """校准力矩传感器零点"""
        ret = self.device.send_cmd(
            self.name, {"command": "calibrate_force", "index": index, "force": force}
        )
        # wrapper_log_message(ret)
        return True

    def set_upload_freq(self, index: int, fps: int) -> bool:
        """设置上传频率"""
        ret = self.device.send_cmd(
            self.name, {"command": "set_upload_frequency", "index": index, "fps": fps}
        )
        # wrapper_log_message(ret)
        return True

    def set_filter_width(self, index: int, width: int) -> bool:
        """设置滤波宽度"""
        ret = self.device.send_cmd(
            self.name, {"command": "set_filter_width", "index": index, "width": width}
        )
        # wrapper_log_message(ret)
        return True
