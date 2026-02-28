from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class PowerControlBoard:

    class StateInfo:
        BATTERY = "battery"
        BOARD_STATE = "state"
        IS_12V_ON = "is_12v_on"
        IS_5V_ON = "is_5v_on"
        IS_REMOTE_CONTROLLER_PAIRED = "is_remote_controller_paired"
        IS_REMOTE_CONTROLLER_CONNECTED = "is_remote_controller_connected"

    class LightEffect:
        OFF = 0
        CONSTANT = 1
        FAST_FLASH = 2
        SLOW_FLASH = 3
        BREATH = 4

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.battery_state = {}
        self.board_state = {}
        self.is_12v_on = False
        self.is_5v_on = False
        self.is_remote_controller_paired = False
        self.is_remote_controller_connected = False

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if (
            state
            and self.name in state
            and state[self.name]["type"] == "power_control_board"
        ):
            self.state = state[self.name]
            ret = True
            if self.StateInfo.BATTERY in self.state:
                self.battery_state = self.state[self.StateInfo.BATTERY]
            else:
                ret = False
            if self.StateInfo.BOARD_STATE in self.state:
                self.board_state = self.state[self.StateInfo.BOARD_STATE]
            else:
                ret = False
            if self.StateInfo.IS_12V_ON in self.state:
                self.is_12v_on = self.state[self.StateInfo.IS_12V_ON]
            else:
                ret = False
            if self.StateInfo.IS_5V_ON in self.state:
                self.is_5v_on = self.state[self.StateInfo.IS_5V_ON]
            else:
                ret = False
            if self.StateInfo.IS_REMOTE_CONTROLLER_PAIRED in self.state:
                self.is_remote_controller_paired = self.state[
                    self.StateInfo.IS_REMOTE_CONTROLLER_PAIRED
                ]
            else:
                ret = False
            if self.StateInfo.IS_REMOTE_CONTROLLER_CONNECTED in self.state:
                self.is_remote_controller_connected = self.state[
                    self.StateInfo.IS_REMOTE_CONTROLLER_CONNECTED
                ]
            else:
                ret = False
            return ret
        return False

    def get_battery_state(self) -> dict:
        """获取电池状态"""
        return self.battery_state

    def get_board_state(self) -> dict:
        """获取缓启板状态"""
        return self.board_state

    def is_12v_power_on(self) -> bool:
        """获取12V电源状态"""
        return self.is_12v_on

    def is_5v_power_on(self) -> bool:
        """获取5V电源状态"""
        return self.is_5v_on

    def is_remote_controller_paired(self) -> bool:
        """获取遥控器配对状态"""
        return self.is_remote_controller_paired

    def is_remote_controller_connected(self) -> bool:
        """获取遥控器连接状态"""
        return self.is_remote_controller_connected

    def get_status(self) -> str:
        """获取缓启板状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查缓启板是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def transport_read(self, address: int, data_size: int) -> list[int]:
        """透传读取缓启板寄存器"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "transport_read",
                "address": address,
                "data_size": data_size,
            },
        )
        return ret

    def transport_write(self, address: int, data_size: int, data: int) -> bool:
        """透传写入缓启板寄存器"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "transport_write",
                "address": address,
                "data_size": data_size,
                "data": data,
            },
        )
        return ret

    def control_front_light(self, brightness: int, effect: LightEffect) -> bool:
        """控制前灯亮度"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "chassis_front_light",
                "brightness": brightness,
                "effect": int(effect),
            },
        )
        return ret

    def control_left_light(self, r: int, g: int, b: int, effect: LightEffect) -> bool:
        """控制左侧灯亮度"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "chassis_left_light",
                "r": r,
                "g": g,
                "b": b,
                "effect": int(effect),
            },
        )
        return ret

    def control_right_light(self, r: int, g: int, b: int, effect: LightEffect) -> bool:
        """控制右侧灯亮度"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "chassis_right_light",
                "r": r,
                "g": g,
                "b": b,
                "effect": int(effect),
            },
        )
        return ret

    def control_chest_light(self, r: int, g: int, b: int, effect: LightEffect) -> bool:
        """控制胸前灯亮度"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "chassis_chest_light",
                "r": r,
                "g": g,
                "b": b,
                "effect": int(effect),
            },
        )
        return ret
