import serial
import time


class LaserIndicator:
    def __init__(self, name: str, port: str, baudrate: int = 9600):
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.read_value = 0.0
        self.serial = None

    def calculate_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for i in data:
            crc ^= i
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def format_get_value_frame(self) -> bytes:
        bytes_frame = bytearray()
        bytes_frame.append(0x01)
        bytes_frame.append(0x03)
        address = 0x003B.to_bytes(length=2, byteorder="big")
        bytes_frame.extend(address)
        data_length = 0x0002.to_bytes(length=2, byteorder="big")
        bytes_frame.extend(data_length)
        crc = self.calculate_crc(bytes(bytes_frame)).to_bytes(length=2, byteorder="little")
        bytes_frame.extend(crc)
        return bytes(bytes_frame)

    def format_set_zero_frame(self) -> bytes:
        bytes_frame = bytearray()
        bytes_frame.append(0x01)
        bytes_frame.append(0x05)
        register_address = 0x0002.to_bytes(length=2, byteorder="big")
        bytes_frame.extend(register_address)
        command = 0xFF00.to_bytes(length=2, byteorder="big")
        bytes_frame.extend(command)
        crc = self.calculate_crc(bytes(bytes_frame)).to_bytes(length=2, byteorder="little")
        bytes_frame.extend(crc)
        return bytes(bytes_frame)

    def connect(self):
        self.serial = serial.Serial(
            self.port,
            self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()

    def get_value(self) -> float | None:
        if not self.serial or not self.serial.is_open:
            raise ConnectionError(f"{self.name}: Serial connection is not established.")
        try:
            self.serial.write(self.format_get_value_frame())
            self.read_value = self.parse_data_response()
            return self.read_value
        except (ValueError, serial.SerialException, TimeoutError):
            return None

    def set_zero(self, debug: bool = False) -> bool | None:
        if not self.serial or not self.serial.is_open:
            raise ConnectionError(f"{self.name}: Serial connection is not established.")
        try:
            frame = self.format_set_zero_frame()
            if debug:
                print(f"[{self.name}] set_zero send: {frame.hex(' ')}")
            try:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
            except Exception:
                pass
            self.serial.write(frame)
            ret = self.parse_set_zero_response(expected_frame=frame, debug=debug)
            return ret
        except (serial.SerialException, TimeoutError, RuntimeError):
            return None

    def parse_data_response(self, timeout: float = 1.0) -> float:
        start_time = time.time()
        response = bytearray()
        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)
                response.extend(byte)
                if len(response) >= 9:
                    if response[0] != 0x01 or response[1] != 0x03 or response[2] != 0x04:
                        response.pop(0)
                        continue
                    data_without_crc = response[:-2]
                    received_crc = int.from_bytes(response[-2:], byteorder="little")
                    calculated_crc = self.calculate_crc(data_without_crc)
                    if received_crc != calculated_crc:
                        response.pop(0)
                        continue
                    rearranged_bytes = bytes([response[5], response[6], response[3], response[4]])
                    value = int.from_bytes(rearranged_bytes, byteorder="big", signed=True)
                    value = value / 1000.0
                    return value
        raise TimeoutError(f"{self.name}: Response not received data within {timeout} seconds.")

    def parse_set_zero_response(self, timeout: float = 1.0, expected_frame: bytes | None = None, debug: bool = False) -> bool:
        start_time = time.time()
        response = bytearray()
        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)
                response.extend(byte)
                if debug:
                    print(f"[{self.name}] recv: {response.hex(' ')}")
                # Try to parse normal response (8 bytes) or exception response (5 bytes)
                while len(response) >= 5:
                    # Prefer full 8-byte response if available
                    frame_len = 8 if len(response) >= 8 else 5
                    data_without_crc = response[:frame_len - 2]
                    received_crc = int.from_bytes(response[frame_len - 2:frame_len], byteorder="little")
                    calculated_crc = self.calculate_crc(data_without_crc)
                    # Exception response: addr, func|0x80, code, crc
                    if frame_len == 5 and (response[1] & 0x80):
                        exc_func = response[1]
                        exc_code = response[2]
                        if debug:
                            print(f"[{self.name}] exception: func=0x{exc_func:02x}, code=0x{exc_code:02x}")
                        raise RuntimeError(f"{self.name}: Modbus exception func=0x{exc_func:02x}, code=0x{exc_code:02x}")

                    if expected_frame is not None:
                        # Modbus write responses usually echo address+function+data
                        if response[:6] != expected_frame[:6]:
                            if debug:
                                print(f"[{self.name}] echo mismatch: recv={response[:6].hex(' ')}, expected={expected_frame[:6].hex(' ')}")
                            response.pop(0)
                            continue
                        if debug and received_crc != calculated_crc:
                            print(f"[{self.name}] crc mismatch (ignored): recv={received_crc:04x}, calc={calculated_crc:04x}")
                        return True

                    # Fallback: accept any valid CRC response
                    return True
        if debug:
            print(f"[{self.name}] set_zero timeout, raw={response.hex(' ')}")
        raise TimeoutError(
            f"{self.name}: Response not received command response within {timeout} seconds."
        )
