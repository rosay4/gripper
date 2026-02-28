"""
简单的日志输出 wrapper。

功能：
- 将日志打印到终端（stdout 或自定义流）
- 可选将日志同步到一个 PyQt 的 info_panel（若提供）
- 管理 info_panel 的最大字符数，超出时会截断最早的内容

实现尽量简单：对 Qt 的依赖为惰性导入
"""

from __future__ import annotations

import sys
import threading
from datetime import datetime
from typing import Optional


class _LogWrapper:
    """日志 wrapper：同时输出到终端和（可选）info_panel。

    用法示例：
            wrapper = LogWrapper(info_panel=my_text_browser, max_chars=200000)
            wrapper.info('started')

    参数:
            info_panel: 可选的 info_panel 实例。
            max_chars: info_panel 中允许的最大字符数，超过时保留最近的部分。
            stream: 可选，写入终端的流，默认 sys.stdout。
    """

    def __init__(
        self,
        info_panel: Optional[object] = None,
        stream=None,
        use_color: bool = True,
    ) -> None:
        self._lock = threading.Lock()
        if info_panel is not None:
            # 惰性导入 InfoPanel（避免在没有 GUI 的环境下强制依赖）
            try:
                from components.ui.info_panel import InfoPanel  # type: ignore
            except Exception:
                # 如果没有 InfoPanel，忽略 info_panel 同步
                return
            self.info_panel = info_panel
        else:
            self.info_panel = None

        self.stream = stream or sys.stdout
        self.use_color = use_color
        print("LogWrapper initialized.")

    # ANSI color codes
    COLORS = {
        "DEBUG": "\033[94m",  # Blue
        "INFO": "\033[92m",  # Green
        "WARN": "\033[93m",  # Yellow
        "ERROR": "\033[91m",  # Red
        "RESET": "\033[0m",  # Reset
    }

    def _format(self, msg: str) -> str:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        return f"{ts} {msg}"

    def log(self, msg: str, level: str = "INFO") -> None:
        """打印一条日志：输出到终端并（若配置）追加到 info_panel。"""
        formatted_for_panel = self._format(f"[{level.upper()}] {str(msg)}")

        color = self.COLORS.get(level.upper(), "") if self.use_color else ""
        reset_color = self.COLORS["RESET"] if color else ""

        formatted_for_terminal = self._format(f"[{level.upper()}] {str(msg)}")

        # 输出到终端（线程安全）
        with self._lock:
            try:
                print(f"{color}{formatted_for_terminal}{reset_color}", file=self.stream)
            except Exception:
                # 如果输出流出错，尽量不抛异常影响调用者
                try:
                    print(f"{color}{formatted_for_terminal}{reset_color}")
                except Exception:
                    pass

        """在主线程中执行：追加文本并在必要时截断 info_panel 内容。"""
        if not self.info_panel:
            return

        # append 会在 info_panel 中增加一行（保留 rich text 支持）
        self.info_panel.log(formatted_for_panel)

    # 方便方法
    def info(self, msg: str) -> None:
        self.log(msg, level="INFO")

    def debug(self, msg: str) -> None:
        self.log(msg, level="DEBUG")

    def warn(self, msg: str) -> None:
        self.log(msg, level="WARN")

    def error(self, msg: str) -> None:
        self.log(msg, level="ERROR")

    def clear_browser(self) -> None:
        """清空绑定的 info_panel（如果存在）。"""
        if not self.info_panel:
            return
        try:
            self.info_panel.clear_log()
        except Exception:
            pass


# 全局 LogWrapper 实例
_global_logger: Optional[_LogWrapper] = None


def initialize_logger_wrapper(info_panel: Optional[object] = None, stream=None) -> None:
    """
    初始化全局 LogWrapper 实例。

    参数:
        info_panel: 可选的 info_panel 实例。
        max_chars: info_panel 中允许的最大字符数，超过时保留最近的部分。
        stream: 可选，写入终端的流，默认 sys.stdout。
    """
    global _global_logger
    _global_logger = _LogWrapper(info_panel, stream)


def log_message(
    *args, level: str = "INFO", sep: str = " ", end: str = "\n"
) -> None:
    """
    记录日志，支持与 print 类似的多参数输入。

    参数:
        *args: 想要打印的内容，可以是多个参数。
        sep: 参数之间的分隔符，默认是空格。
        end: 日志结尾的字符，默认是换行符。
    """
    global _global_logger

    # 将参数拼接成字符串
    msg = sep.join(map(str, args)) + end

    if _global_logger is None:
        # 如果未初始化，直接调用 print
        print(msg, end="")
    else:
        _global_logger.log(msg.strip(), level=level)


__all__ = ["initialize_logger_wrapper", "log_message"]


if __name__ == "__main__":
    # 简单测试
    # 初始化全局日志包装器，绑定到 QTextBrowser
    initialize_logger_wrapper(None)

    # 记录一些日志
    for i in range(5000):
        log_message("Log message", i)

    sys.exit(0)
