from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class IOBox:

    class StateInfo:
        IO_STATUS = "io_status"

    class IOKey:
        HUB_POWER = "Hub power"
        CAMERA_POWER = "Camera power"
        MOTOR_POWER = "Motor power"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.io_status = {}
        self.hub_power: bool = False
        self.camera_power: bool = False
        self.motor_power: bool = False

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if state and self.name in state and state[self.name]["type"] == "io_control":
            self.state = state[self.name]
            ret = True
            if self.StateInfo.IO_STATUS in self.state:
                self.io_status = self.state[self.StateInfo.IO_STATUS]
                # wrapper_log_message(f"IO Box {self.name} status: {self.io_status}")
            else:
                ret = False
            return ret
        return False

    def get_hub_power(self) -> bool:
        """获取集线器电源状态"""
        for name, status in self.io_status:
            if name == self.IOKey.HUB_POWER:
                return status
        return False

    def set_hub_power(self, power: bool) -> bool:
        """设置集线器电源状态"""
        ret = self.device.send_cmd(
            self.name,
            {"command": "set_io", "name": self.IOKey.HUB_POWER, "value": power},
        )
        return True

    def get_camera_power(self) -> bool:
        """获取相机电源状态"""
        for name, status in self.io_status:
            if name == self.IOKey.CAMERA_POWER:
                return status
        return False

    def set_camera_power(self, power: bool) -> bool:
        """设置相机电源状态"""
        ret = self.device.send_cmd(
            self.name,
            {"command": "set_io", "name": self.IOKey.CAMERA_POWER, "value": power},
        )
        return True

    def get_motor_power(self) -> bool:
        """获取电机电源状态"""
        for name, status in self.io_status:
            if name == self.IOKey.MOTOR_POWER:
                return status
        return False

    def set_motor_power(self, power: bool) -> bool:
        """设置电机电源状态"""
        ret = self.device.send_cmd(
            self.name,
            {"command": "set_io", "name": self.IOKey.MOTOR_POWER, "value": power},
        )
        return True

    def get_status(self) -> str:
        """获取IO盒子状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查IO盒子是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None
