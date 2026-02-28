from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class Wrench:

    class StateInfo:
        FORCE = "force"
        TORQUE = "torque"
        CALIBRATED = "calibrated"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.force: list[float] = []
        self.torque: list[float] = []
        self.calibrated: bool = False

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if state and self.name in state and state[self.name]["type"] == "wrench":
            self.state = state[self.name]
            ret = True
            if self.StateInfo.FORCE in self.state:
                self.force = self.state[self.StateInfo.FORCE]
            else:
                ret = False
            if self.StateInfo.TORQUE in self.state:
                self.torque = self.state[self.StateInfo.TORQUE]
            else:
                ret = False
            if self.StateInfo.CALIBRATED in self.state:
                self.calibrated = self.state[self.StateInfo.CALIBRATED]
            else:
                ret = False
            return ret
        return False

    def get_force(self) -> tuple[bool, list[float]]:
        """获取当前力"""
        if self.get_state():
            return True, self.force

        return False, []

    def get_last_force(self) -> list[float]:
        """获取当前缓存的力"""
        return self.force

    def get_torque(self) -> tuple[bool, list[float]]:
        """获取当前力矩"""
        if self.get_state():
            return True, self.torque

  

    def get_last_torque(self) -> list[float]:
        """获取当前缓存的力矩"""
        return self.torque

    def get_calibrated_flag(self) -> bool:
        """获取是否已校准"""
        if self.get_state():
            return self.calibrated

        return False

    def get_last_calibrated_flag(self) -> bool:
        """获取当前缓存的校准状态"""
        return self.calibrated

    def set_id(self, id: int) -> bool:
        """设置传感器ID"""
        return False  # 目前不支持

    def get_status(self) -> str:
        """获取传感器状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查六维力传感器是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def reset_wrench(self) -> bool:
        """重置力矩传感器"""
        ret = self.device.send_cmd(self.name, {"command": "reset_sensor"})
        # wrapper_log_message(ret)
        return True

    def calibrate_wrench(self) -> bool:
        """校准力矩传感器"""
        ret = self.device.send_cmd(self.name, {"command": "calibration"})
        # wrapper_log_message(ret)
        return True