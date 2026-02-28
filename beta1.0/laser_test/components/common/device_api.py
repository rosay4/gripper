import threading
import time
from abc import ABC, abstractmethod
from importlib.metadata import version, PackageNotFoundError

# import pathlib
# import json
import rb_python
import hblog

from components.common.logger_wrapper import log_message

RB_PYTHON_NAME = "rb_python"
RB_PYTHON_MIN_VERSION = (0, 45, 26)
HBLOG_NAME = "hblog"
HBLOG_MIN_VERSION = (0, 2, 0)
HID_CONTROLLER_NAME = "hid_controller"
HID_CONTROLLER_MIN_VERSION = (0, 0, 1)


class Device:
    class Status:
        Starting = "starting"
        Operational = "operational"
        Failed = "failed"

    def __init__(self, cfg_str, log_level):
        # 检查依赖版本
        self._check_dependency_versions()

        self.robot = None

        self.states = {}
        self.errors = {}
        self.cfg_str = cfg_str
        Device._init_hblog(log_level)

    def _init_hblog(log_level):

        valid_log_levels = {"debug", "info", "warn", "error", "fatal"}

        if log_level not in valid_log_levels:
            log_message(
                f"Invalid log level: {log_level}. Must be one of {valid_log_levels}."
            )
            log_level = "warn"

        config = {
            "refresh_rate": "30 seconds",
            "appenders": {
                "stderr": {
                    "kind": "console",
                    "target": "stderr",
                    "encoder": {
                        "pattern": "{h({d(%Y-%m-%d %H:%M:%S.%3f)} [{t}] {l} {m})}{n}"
                    },
                },
                "file": {
                    "kind": "file",
                    "path": "log/file.log",
                    "encoder": {
                        "pattern": "{h({d(%Y-%m-%d %H:%M:%S.%3f)} [{t}] {l} {m})}{n}"
                    },
                },
            },
            "root": {"level": log_level, "appenders": ["stderr", "file"]},
            "loggers": {},
        }
        try:
            hblog.start(config)
        except Exception as e:
            log_message(f"Failed to start hblog with config: {config}, error: {e}", 'warn')

    def _check_dependency_versions(self):
        """检查依赖包的版本是否符合要求"""
        dependencies = {
            RB_PYTHON_NAME: RB_PYTHON_MIN_VERSION,  # 最低版本
            HBLOG_NAME: HBLOG_MIN_VERSION,
            HID_CONTROLLER_NAME: HID_CONTROLLER_MIN_VERSION,
        }

        for package, min_version in dependencies.items():
            try:
                installed_version = tuple(map(int, version(package).split(".")))
                if installed_version < min_version:
                    raise RuntimeError(
                        f"版本冲突: {package} 需要 >= {'.'.join(map(str, min_version))}, "
                        f"但当前版本为 {'.'.join(map(str, installed_version))}"
                    )
                else:
                    log_message(
                        f"{package} 版本检查通过: {'.'.join(map(str, installed_version))}"
                    )
            except PackageNotFoundError:
                raise RuntimeError(f"未找到依赖包: {package}")

    def __del__(self):
        """析构函数，确保驱动正确退出"""
        self.shutdown()

    @abstractmethod
    def set_action(self, action):
        """设置驱动器的动作"""
        raise NotImplementedError("请在子类中实现此方法")

    @abstractmethod
    def get_states(self) -> dict:
        """获取驱动器的状态"""
        raise NotImplementedError("请在子类中实现此方法")

    @abstractmethod
    def send_cmd(self, name, cmd):
        """发送命令到驱动器"""
        raise NotImplementedError("请在子类中实现此方法")

    @abstractmethod
    def get_errors(self) -> dict:
        """获取驱动器的错误状态"""
        raise NotImplementedError("请在子类中实现此方法")

    @abstractmethod
    def get_states(self) -> dict:
        """获取驱动器的状态"""
        raise NotImplementedError("请在子类中实现此方法")

    @abstractmethod
    def shutdown(self):
        pass

    def get_cfg(self):
        """获取当前配置"""
        return self.cfg_str


class LocalDevice(Device):
    def __init__(self, cfg_str, log_level="warn"):
        super().__init__(cfg_str, log_level)
        # 实例化Robot
        self.robot = rb_python.robot.Robot(cfg_str)

        log_message("LocalDevice initialized")

    def set_action(self, action):
        self.robot.set_actions(action)
        # wrapper_log_message(f"LocalDevice set_action called with action: {action}")

    def get_states(self, hw: list[str]) -> dict:
        # wrapper_log_message("LocalDevice get_states called")
        return self.robot.get_states(hw)

    def send_cmd(self, name, cmd):
        # wrapper_log_message(f"LocalDevice send_cmd called with cmd:{name}, {cmd}")
        return self.robot.send_command(name, cmd)

    def get_errors(self) -> dict:
        # wrapper_log_message("LocalDevice get_error called")
        return self.robot.get_errors()

    def get_status(self) -> dict:
        # wrapper_log_message("LocalDevice get_status called")
        return self.robot.get_status()

    def shutdown(self):
        if self.robot:
            self.robot.shutdown()
        log_message("LocalDevice shutdown completed")

class RemoteDevice(Device):
    def __init__(
        self, cfg_str, ip: str, port: int, log_level="warn"
    ):
        super().__init__(cfg_str, log_level)
        self.ip = ip
        self.port = port
        # 初始化网络连接
        log_message(f"RemoteDevice initialized with IP: {ip}, Port: {port}")

    def set_action(self, action):
        log_message(f"RemoteDevice set_action called with action: {action}")

    def get_states(self, hw: list[str]) -> dict:
        log_message("RemoteDevice get_state called")
        return None

    def send_cmd(self, name, cmd):
        log_message(f"RemoteDevice send_cmd called with cmd: {cmd}")

    def get_errors(self) -> dict:
        log_message("RemoteDevice get_error called")
        return None

    def get_status(self, name) -> dict:
        log_message("RemoteDevice get_status called")
        return None
