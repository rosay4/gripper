from components.common.device_api import LocalDevice, RemoteDevice, Device
from components.common.logger_wrapper import log_message


class Base:

    class BaseMode:
        RUN = "running_mode"
        CALIBRATION = "calibration_mode"

    class StateInfo:
        TWIST = "twist"
        ODOM = "odom"
        WHEEL_STEERING = "wheel_steering"
        WHEEL_VELOCITY = "wheel_velocity"
        STEERING_MOTOR = "steering_motor"
        DRIVING_MOTOR = "driving_motor"
        MOTOR_POSITION = "position"
        MOTOR_VELOCITY = "velocity"
        MOTOR_TORQUE = "torque"

    def __init__(self, name: str, device: LocalDevice | RemoteDevice):
        self.name = name
        self.device = device
        self.twist: list[float] = []
        self.target_twist: list[float] = [0.0, 0.0, 0.0]
        self.odom: list[float] = []
        self.steering_position: list[float] = []
        self.steering_velocity: list[float] = []
        self.steering_torque: list[float] = []
        self.steering_temperature: list[float] = []
        self.driving_position: list[float] = []
        self.driving_velocity: list[float] = []
        self.driving_torque: list[float] = []
        self.driving_temperature: list[float] = []
        self.base_mode = self.BaseMode.RUN  # 默认模式
    
    def set_twist(self, twist: list[float]) -> bool:
        """设置底盘速度指令"""
        if not twist or len(twist) != 3:
            log_message(f"Error: Invalid twist {twist}")
            return False

        action = {self.name: {"type": "velocity", "velocity": twist}}
        self.device.set_action(action)
        self.target_twist = twist
        # wrapper_log_message(ret)
        return True

    def get_target_twist(self) -> list[float]:
        """获取当前目标速度指令"""
        return self.target_twist

    def get_state(self) -> bool:
        """获取当前状态"""
        state = self.device.get_states([self.name])
        # wrapper_log_message(state)
        if state and self.name in state and state[self.name]["type"] == "base":
            self.state = state[self.name]
            ret = True
            if self.StateInfo.TWIST in self.state:
                self.twist = list(self.state[self.StateInfo.TWIST])
            else:
                ret = False
            if self.StateInfo.ODOM in self.state:
                self.odom = list(self.state[self.StateInfo.ODOM])
            else:
                ret = False
            if self.StateInfo.STEERING_MOTOR in self.state:
                steering_motor = self.state[self.StateInfo.STEERING_MOTOR]
                if self.StateInfo.MOTOR_POSITION in steering_motor:
                    self.steering_position = steering_motor[
                        self.StateInfo.MOTOR_POSITION
                    ]
                else:
                    ret = False

                if self.StateInfo.MOTOR_VELOCITY in steering_motor:
                    self.steering_velocity = steering_motor[
                        self.StateInfo.MOTOR_VELOCITY
                    ]
                else:
                    ret = False

                if self.StateInfo.MOTOR_TORQUE in steering_motor:
                    self.steering_torque = steering_motor[self.StateInfo.MOTOR_TORQUE]
                else:
                    ret = False
            else:
                ret = False

            if self.StateInfo.DRIVING_MOTOR in self.state:
                driving_motor = self.state[self.StateInfo.DRIVING_MOTOR]
                if self.StateInfo.MOTOR_POSITION in driving_motor:
                    self.driving_position = driving_motor[self.StateInfo.MOTOR_POSITION]
                else:
                    ret = False

                if self.StateInfo.MOTOR_VELOCITY in driving_motor:
                    self.driving_velocity = driving_motor[self.StateInfo.MOTOR_VELOCITY]
                else:
                    ret = False

                if self.StateInfo.MOTOR_TORQUE in driving_motor:
                    self.driving_torque = driving_motor[self.StateInfo.MOTOR_TORQUE]
                else:
                    ret = False
            else:
                ret = False

            return ret
        return False

    def get_twist(self) -> tuple[bool, list[float]]:
        """获取当前速度"""
        if self.get_state():
            return True, self.twist

        return False, []

    def get_last_twist(self) -> list[float]:
        """获取当前缓存的速度"""
        return self.twist

    def get_odom(self) -> tuple[bool, list[float]]:
        """获取当前里程计"""
        if self.get_state():
            return True, self.odom

        return False, []

    def get_last_odom(self) -> list[float]:
        """获取当前缓存的里程计"""
        return self.odom

    def get_steering_position(self) -> tuple[bool, list[float]]:
        """获取当前位置"""
        if self.get_state():
            return True, self.steering_position

        return False, []

    def get_last_steering_position(self) -> list[float]:
        """获取当前缓存的位置"""
        return self.steering_position

    def get_driving_position(self) -> tuple[bool, list[float]]:
        """获取当前位置"""
        if self.get_state():
            return True, self.driving_position

        return False, []

    def get_last_driving_position(self) -> list[float]:
        """获取当前缓存的位置"""
        return self.driving_position

    def get_steering_velocity(self) -> tuple[bool, list[float]]:
        """获取当前速度"""
        if self.get_state():
            return True, self.steering_velocity

        return False, []

    def get_last_steering_velocity(self) -> list[float]:
        """获取当前缓存的速度"""
        return self.steering_velocity

    def get_driving_velocity(self) -> tuple[bool, list[float]]:
        """获取当前速度"""
        if self.get_state():
            return True, self.driving_velocity

        return False, []

    def get_last_driving_velocity(self) -> list[float]:
        """获取当前缓存的速度"""
        return self.driving_velocity

    def get_steering_torque(self) -> tuple[bool, list[float]]:
        """获取当前力矩"""
        if self.get_state():
            return True, self.steering_torque

        return False, []

    def get_last_steering_torque(self) -> list[float]:
        """获取当前缓存的力矩"""
        return self.steering_torque

    def get_driving_torque(self) -> tuple[bool, list[float]]:
        """获取当前力矩"""
        if self.get_state():
            return True, self.driving_torque

        return False, []

    def get_last_driving_torque(self) -> list[float]:
        """获取当前缓存的力矩"""
        return self.driving_torque

    def get_steering_temperature(self) -> tuple[bool, list[float]]:
        """获取当前温度"""
        ret = self.device.send_cmd(self.name, {"command": "get_steering_temperature"})
        # wrapper_log_message(ret)
        if "value" in ret:
            self.steering_temperature = ret["value"]
            return True, self.steering_temperature
        return False, []

    def get_last_steering_temperature(self) -> list[float]:
        """获取当前缓存的温度"""
        return self.steering_temperature

    def get_driving_temperature(self) -> tuple[bool, list[float]]:
        """获取当前温度"""
        ret = self.device.send_cmd(self.name, {"command": "get_driving_temperature"})
        # wrapper_log_message(ret)
        if "value" in ret:
            self.driving_temperature = ret["value"]
            return True, self.driving_temperature
        return False, []

    def get_last_driving_temperature(self) -> list[float]:
        """获取当前缓存温度"""
        return self.driving_temperature

    def get_mode(self) -> str:
        """获取当前底盘模式"""
        mode = self.device.send_cmd(self.name, {"command": "get_control_mode"})["mode"]
        return mode

    def set_mode(self, mode: str) -> bool:
        """设置底盘模式"""
        if mode not in (
            self.BaseMode.RUN,
            self.BaseMode.CALIBRATION,
        ):
            log_message(f"Error: Invalid motor mode {mode}")
            return False

        self.motor_mode = mode
        ret = self.device.send_cmd(
            self.name, {"command": "set_control_mode", "mode": mode}
        )
        # wrapper_log_message(ret)
        return True

    def get_status(self) -> str:
        """获取底盘状态"""
        status = self.device.get_status()
        if status and self.name in status:
            return status[self.name]
        return None

    def is_operational(self) -> bool:
        """检查底盘是否处于 operational 状态"""
        status = self.get_status()
        return status == Device.Status.Operational

    def get_error(self) -> tuple[list[float], list[str]]:
        """获取报错信息"""
        errors = self.device.get_errors()
        if errors and self.name in errors:
            # wrapper_log_message(f"Motor {self.name} errors: {errors[self.name]}")
            return errors[self.name]["code"], errors[self.name]["str"]
        return None, None

    def reset_base(self) -> bool:
        """重置底盘"""
        ret = self.device.send_cmd(self.name, {"command": "reset_base"})
        # wrapper_log_message(ret)
        return True

    def calibrate(self) -> bool:
        """标定底盘"""
        ret = self.device.send_cmd(self.name, {"command": "set_steering_zero"})
        # wrapper_log_message(ret)
        return True

    def cancel_limit(self) -> bool:
        """取消底盘限位"""
        ret = self.device.send_cmd(self.name, {"command": "cancel_steering_limit"})
        # wrapper_log_message(ret)
        return True

    def calibrate_degrees(self, indexs: list[int], degrees: list[float]) -> bool:
        """标定底盘指定轮子角度"""
        ret = self.device.send_cmd(
            self.name,
            {
                "command": "set_steering_degree",
                "indexs": indexs,
                "degrees": degrees,
            },
        )
        # wrapper_log_message(ret)
        return True
