import os
import csv
import time
from typing import List, Union
from enum import Enum
from components.common.logger_wrapper import log_message


class DataLoggerByLineCount:
    """
    数据记录器，用于将数据按批次写入CSV文件或一行一行读取数据
    """
    class DataLoggerMode(Enum):
        READ = "read"
        WRITE = "write"

    def __init__(self, file_path: str, column_names: List[str], batch_size: int, mode: DataLoggerMode):
        """
        初始化数据记录器

        :param file_path: 文件路径（READ 模式下为具体文件路径，WRITE 模式下为文件夹路径）
        :param column_names: 列名列表
        :param batch_size: 每多少条数据输出一次文件
        :param mode: 模式，DataLoggerMode.READ 或 DataLoggerMode.WRITE
        :raises ValueError: 当 mode 非法时
        """
        if not isinstance(mode, self.DataLoggerMode):
            raise ValueError("Mode must be an instance of DataLoggerMode")

        self.file_path = file_path
        self.column_names = column_names
        self.batch_size = batch_size
        self.mode = mode
        self.data_buffer: List[List[Union[str, int, float]]] = []
        self.file_counter = 0
        self.total_rows = 0
        self.total_columns = len(column_names)
        self._read_file = None

        if self.mode == self.DataLoggerMode.WRITE:
            # 创建文件夹（如果不存在）
            os.makedirs(self.file_path, exist_ok=True)
            log_message(f"Start logging to {self.file_path}")
        elif self.mode == self.DataLoggerMode.READ:
            self.file_iterator = self._get_file_iterator()
            self._calculate_file_dimensions()

    def _get_file_iterator(self):
        """
        获取文件迭代器，用于读取指定文件中的数据
        """
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"File {self.file_path} does not exist")

        csvfile = open(self.file_path, "r", newline="", encoding="utf-8")
        self._read_file = csvfile
        log_message(f"Opened file {self.file_path} for reading, handle {self._read_file.fileno}.")

        def iterator():
            try:
                reader = csv.reader(csvfile)
                # 跳过列名
                next(reader, None)
                for row in reader:
                    yield row
            finally:
                self._close_reader()

        return iterator()

    def _calculate_file_dimensions(self):
        """
        计算文件的总行数和列数
        """
        if not os.path.exists(self.file_path):
            raise FileNotFoundError(f"File {self.file_path} does not exist")

        with open(self.file_path, "r", newline="", encoding="utf-8") as csvfile:
            reader = csv.reader(csvfile)
            # 跳过列名
            header = next(reader, None)
            if header:
                self.total_columns = len(header)
            self.total_rows = sum(1 for _ in reader)

        log_message(
            f"File {self.file_path} has {self.total_rows} rows and {self.total_columns} columns."
        )

    def read_next(self) -> Union[List[str], None]:
        """
        读取下一行数据

        :return: 下一行数据，或者 None 如果没有更多数据
        :raises RuntimeError: 当实例不在 DataLoggerMode.READ 模式时
        """
        if self.mode != self.DataLoggerMode.READ:
            raise RuntimeError("DataLogger is not in 'read' mode")

        if self.file_iterator is None:
            return None

        try:
            return next(self.file_iterator)
        except StopIteration:
            self.close()
            return None

    def record(self, data: List[Union[str, int, float]]) -> None:
        """
        记录一条数据

        :param data: 与列数相同的数组，表示一条数据
        :raises ValueError: 当数据长度与列数不匹配时
        :raises RuntimeError: 当实例不在 DataLoggerMode.WRITE 模式时
        """
        if self.mode != self.DataLoggerMode.WRITE:
            raise RuntimeError("DataLogger is not in 'write' mode")

        if len(data) != len(self.column_names):
            log_message(
                f"({self.file_path})数据长度({len(data)})与列数({len(self.column_names)})不匹配"
            )
            return

        self.data_buffer.append(data)

        # 当数据量达到批次大小时，写入文件
        if self.batch_size > 0 and len(self.data_buffer) >= self.batch_size:
            self._write_batch()

    def _write_batch(self) -> None:
        """
        将缓冲区的数据写入CSV文件
        """
        if not self.data_buffer:
            return

        time = self.data_buffer[0][0]
        file_path = os.path.join(
            self.file_path,
            f"{time}.csv",
        )

        # 检查文件是否存在，如果不存在则写入列名
        file_exists = os.path.exists(file_path)

        with open(file_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)

            # 如果文件不存在，则写入列名
            if not file_exists:
                writer.writerow(self.column_names)

            # 写入数据
            writer.writerows(self.data_buffer)

        # 清空缓冲区并增加文件计数器
        self.data_buffer.clear()
        self.file_counter += 1
        log_message(
            f"Write to file {file_path}, totally {self.file_counter} files"
        )

    def _write_row_to_file(self, row: List[Union[str, int, float]]) -> None:
        if not self.data_buffer:
            return

        time = self.data_buffer[0][0]
        file_path = os.path.join(
            self.file_path,
            f"{time}.csv",
        )

        # 检查文件是否存在，如果不存在则写入列名
        file_exists = os.path.exists(file_path)

        with open(file_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)

            # 如果文件不存在，则写入列名
            if not file_exists:
                writer.writerow(self.column_names)

            # 写入数据
            writer.writerows(row)

        log_message(
            f"Write to file {file_path}"
        )

    def finish(self) -> None:
        """
        结束记录，将剩余数据写入文件
        :raises RuntimeError: 当实例不在 DataLoggerMode.WRITE 模式时
        """
        if self.mode != self.DataLoggerMode.WRITE:
            raise RuntimeError("DataLogger is not in 'write' mode")

        log_message(f"Finish logging.")
        if self.data_buffer:
            self._write_batch()

    def get_current_directory(self) -> str:
        """
        获取当前日志文件夹路径

        :return: 当前日志文件夹路径
        """
        return self.file_path

    def close(self) -> None:
        """关闭读取模式下的文件句柄。"""
        if self.mode == self.DataLoggerMode.WRITE:
            return

        if self.file_iterator is not None:
            try:
                self.file_iterator.close()  # type: ignore[attr-defined]
                log_message("Closed read file iterator.")
            except Exception:
                pass
            finally:
                self.file_iterator = None

        self._close_reader()

    def _close_reader(self) -> None:
        if self._read_file is not None:
            try:
                self._read_file.close()
                log_message("Closed reader.")
            finally:
                self._read_file = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


class DataLoggerByTime:
    """
    数据记录器，根据时间间隔切分文件，用于将数据写入CSV文件
    """
    class DataLoggerMode(Enum):
        READ = "read"
        WRITE = "write"

    def __init__(self, file_path: str, column_names: List[str], time_interval: int, mode: DataLoggerMode):
        """
        初始化数据记录器

        :param file_path: 文件夹路径（WRITE 模式下）
        :param column_names: 列名列表
        :param time_interval: 时间间隔（秒），每隔多少秒切分一个文件
        :param mode: 模式，DataLoggerMode.WRITE
        :raises ValueError: 当 mode 非法时
        """
        if not isinstance(mode, self.DataLoggerMode):
            raise ValueError("Mode must be an instance of DataLoggerMode")

        self.file_path = file_path
        self.column_names = column_names
        self.time_interval = time_interval
        self.mode = mode
        self.data_buffer: List[List[Union[str, int, float]]] = []
        self.current_file_time = None
        self.current_file_path = None

        if self.mode == self.DataLoggerMode.WRITE:
            # 创建文件夹（如果不存在）
            os.makedirs(self.file_path, exist_ok=True)
            log_message(f"Start logging to {self.file_path} with time interval {self.time_interval}s")
        else:
            raise ValueError("DataLoggerByTime only supports WRITE mode")

    def record(self, data: List[Union[str, int, float]], flush_now: bool = False) -> None:
        """
        记录一条数据

        :param data: 与列数相同的数组，表示一条数据
        :param flush_now: 是否立即将当前缓存写入文件（不切换文件）
        :raises ValueError: 当数据长度与列数不匹配时
        :raises RuntimeError: 当实例不在 DataLoggerMode.WRITE 模式时
        """
        if self.mode != self.DataLoggerMode.WRITE:
            raise RuntimeError("DataLogger is not in 'write' mode")

        if len(data) != len(self.column_names):
            log_message(
                f"({self.file_path})数据长度({len(data)})与列数({len(self.column_names)})不匹配"
            )
            return

        self.data_buffer.append(data)

        # 如果需要立即刷新，写入当前文件
        if flush_now:
            self._write_to_current_file()

        # 检查是否需要切分文件
        current_time = time.time()
        if self.current_file_time is None or current_time - self.current_file_time >= self.time_interval:
            self._write_batch_and_switch_file()

    def _write_to_current_file(self) -> None:
        """
        将缓冲区的数据写入当前文件（不切换文件）
        """
        if not self.data_buffer or self.current_file_path is None:
            return

        file_exists = os.path.exists(self.current_file_path)

        with open(self.current_file_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)

            # 如果文件不存在，则写入列名
            if not file_exists:
                writer.writerow(self.column_names)

            # 写入数据
            writer.writerows(self.data_buffer)

        # 清空缓冲区
        self.data_buffer.clear()
        log_message(f"Flushed buffer to current file {self.current_file_path}")

    def _write_batch_and_switch_file(self) -> None:
        """
        将缓冲区的数据写入新文件并切换文件
        """
        if not self.data_buffer:
            return

        current_time = time.time()
        # 计算文件时间戳（向下取整到时间间隔）
        file_time = int(current_time // self.time_interval) * self.time_interval
        file_path = os.path.join(
            self.file_path,
            f"{file_time}.csv",
        )

        # 如果是新文件，更新当前文件
        if self.current_file_path != file_path:
            self.current_file_time = file_time
            self.current_file_path = file_path

        file_exists = os.path.exists(file_path)

        with open(file_path, "a", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)

            # 如果文件不存在，则写入列名
            if not file_exists:
                writer.writerow(self.column_names)

            # 写入数据
            writer.writerows(self.data_buffer)

        # 清空缓冲区
        self.data_buffer.clear()
        log_message(f"Switched to new file {file_path}")

    def finish(self) -> None:
        """
        结束记录，将剩余数据写入文件
        """
        if self.mode != self.DataLoggerMode.WRITE:
            raise RuntimeError("DataLogger is not in 'write' mode")

        log_message("Finish logging.")
        if self.data_buffer:
            self._write_batch_and_switch_file()

    def get_current_directory(self) -> str:
        """
        获取当前日志文件夹路径

        :return: 当前日志文件夹路径
        """
        return self.file_path

    def __del__(self) -> None:
        try:
            self.finish()
        except Exception:
            pass


if __name__ == "__main__":
    """
    数据记录器使用示例
    """
    # 创建数据记录器
    # 参数：文件夹名，列名列表，每批次数据量
    logger = DataLoggerByLineCount("./logs", ["timestamp", "temperature", "humidity"], 5, DataLoggerByLineCount.DataLoggerMode.WRITE)

    # 模拟记录数据
    for i in range(12):
        # 模拟传感器数据
        data = [time.time(), 20 + i * 0.5, 50 + i * 0.3]
        logger.record(data)
        log_message(f"记录数据 {i+1}: {data}")

        time.sleep(0.1)  # 模拟数据采集间隔

    # 完成记录，确保所有数据都写入文件
    logger.finish()
    log_message("数据记录完成")

    # 读取数据示例
    for i in range(1000):
        logger_read = DataLoggerByLineCount("./logs/1761706484.5299592.csv", ["timestamp", "temperature", "humidity"], 5, DataLoggerByLineCount.DataLoggerMode.READ)
        for i in range(12):
            data = logger_read.read_next()
            if data is not None:
                log_message(f"读取数据 {i+1}: {data}")
            else:
                break
        logger_read.close()

    logger = DataLoggerByTime("./time_logs", ["timestamp", "temperature", "humidity"], 10, DataLoggerByTime.DataLoggerMode.WRITE)

    # 模拟记录数据
    for i in range(25):
        # 模拟传感器数据
        data = [time.time(), 20 + i * 0.5, 50 + i * 0.3]
        logger.record(data)
        log_message(f"记录数据 {i+1}: {data}")

        time.sleep(1)  # 模拟数据采集间隔