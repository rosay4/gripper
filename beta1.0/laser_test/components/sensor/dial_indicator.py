import serial
import time


class DialIndicator:
    def __init__(self, name: str, port: str, baudrate: int = 38400):
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.read_value = 0.0
        self.serial = None

    def calculate_crc(self, data: bytes) -> int:
        crc = 0xFFFF
        for i in data:
            crc ^= i
            for j in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1

        return crc

    def format_get_value_frame(self) -> bytes:
        """Generate the frame to request the current value."""
        bytes_frame = bytearray()
        bytes_frame.append(0x01)  # Address code
        bytes_frame.append(0x03)  # Function code

        address = 0x0000.to_bytes(length=2, byteorder='big')  # Starting address
        bytes_frame.extend(address)
        # bytes_frame.append((address >> 8) & 0xFF)  # High byte of address
        # bytes_frame.append(address & 0xFF)  # Low byte of address

        data_length = 0x0002.to_bytes(length=2, byteorder='big')  # Length of data to read
        bytes_frame.extend(data_length)
        # bytes_frame.append((data_length >> 8) & 0xFF)  # High byte of length
        # bytes_frame.append(data_length & 0xFF)  # Low byte of length

        # Calculate CRC
        crc = self.calculate_crc(bytes(bytes_frame)).to_bytes(length=2, byteorder='little')
        bytes_frame.extend(crc)
        # bytes_frame.append(crc & 0xFF)  # Low byte of CRC
        # bytes_frame.append((crc >> 8) & 0xFF)  # High byte of CRC

        # print(f"Get Value Frame: {[hex(b) for b in bytes_frame]}")
        return bytes(bytes_frame)

    def format_set_zero_frame(self) -> bytes:
        """Generate the frame to set the current value to zero."""
        bytes_frame = bytearray()
        bytes_frame.append(0x01)  # Address code
        bytes_frame.append(0x06)  # Function code

        register_address = 0x0800.to_bytes(length=2, byteorder='big')  # Register address
        bytes_frame.extend(register_address)
        # bytes_frame.append((register_address >> 8) & 0xFF)  # High byte of register address
        # bytes_frame.append(register_address & 0xFF)  # Low byte of register address

        command = 0xAB56.to_bytes(length=2, byteorder='big')  # Command to set zero
        bytes_frame.extend(command)
        # bytes_frame.append((command >> 8) & 0xFF)  # High byte of command
        # bytes_frame.append(command & 0xFF)  # Low byte of command

        # Calculate CRC
        crc = self.calculate_crc(bytes(bytes_frame)).to_bytes(length=2, byteorder='little')
        bytes_frame.extend(crc)
        # bytes_frame.append(crc & 0xFF)  # Low byte of CRC
        # bytes_frame.append((crc >> 8) & 0xFF)  # High byte of CRC

        print(f"Set Zero Frame: {[hex(b) for b in bytes_frame]}")
        return bytes(bytes_frame)

    def connect(self):
        """Establish a serial connection."""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
            )
            # For RS485, you may need to enable RS485 mode.
            # This depends on your hardware and driver.
            # If you have an adapter that automatically handles line direction,
            # you may not need this.
            # self.serial.rs485_mode = serial.rs485.RS485Settings()
            print(f"{self.name}: Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"{self.name}: Failed to connect to {self.port}. Error: {e}")
            raise

    def disconnect(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"{self.name}: Disconnected from {self.port}.")

    def get_value(self) -> float | None:
        """Send a command to get the current value and parse the response."""
        if not self.serial or not self.serial.is_open:
            raise ConnectionError(f"{self.name}: Serial connection is not established.")

        try:
            self.serial.write(self.format_get_value_frame())  # Send command

            self.read_value = self.parse_data_response()  # Parse the response

            # print(f"{self.name}: Received value {self.read_value}.")
            return self.read_value
        except (ValueError, serial.SerialException, TimeoutError) as e:
            print(f"{self.name}: Failed to get value. Error: {e}")
            return None

    def set_zero(self) -> bool | None:
        """Send a command to set the current value to zero."""
        if not self.serial or not self.serial.is_open:
            raise ConnectionError(f"{self.name}: Serial connection is not established.")

        try:
            self.serial.write(self.format_set_zero_frame())  # Send command

            ret = self.parse_set_zero_response()  # Parse the response
            return ret
        except (serial.SerialException, TimeoutError) as e:
            print(f"{self.name}: Failed to set zero. Error: {e}")
            return None

    def parse_data_response(self, timeout: float = 1.0) -> float:
        """Parse the response byte by byte to handle potential packet sticking.

        Args:
            timeout (float): Maximum time to wait for a complete response, in seconds.

        Returns:
            float: The parsed value from the response.

        Raises:
            TimeoutError: If the response is not received within the timeout period.
            ValueError: If the response is invalid or cannot be parsed.
        """
        import time

        start_time = time.time()
        response = bytearray()

        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)  # Read one byte at a time
                response.extend(byte)

                # Check if we have received at least the minimum packet length
                if (
                    len(response) >= 9
                ):  # Address code (1) + Func (1) + DataLen (1) + Data (4) + CRC (2)
                    if (
                        response[0] != 0x01
                        or response[1] != 0x03
                        or response[2] != 0x04
                    ):
                        response.pop(0)  # Remove the first byte and continue
                        continue

                    # Validate CRC
                    data_without_crc = response[:-2]
                    received_crc = int.from_bytes(response[-2:], byteorder="little")
                    calculated_crc = self.calculate_crc(data_without_crc)

                    if received_crc != calculated_crc:
                        response.pop(0)  # Remove the first byte and continue
                        print(f"{self.name}: CRC mismatch. Expected {calculated_crc}, got {received_crc}.")
                        continue

                    # Extract the value from the data field
                    value = int.from_bytes(response[3:7], byteorder="big", signed=False)
                    value = value / 1000.0  # Change value to mm

                    return value

        raise TimeoutError(
            f"{self.name}: Response not received data within {timeout} seconds."
        )

    def parse_set_zero_response(self, timeout: float = 1.0) -> bool:
        """Parse the response byte by byte to handle potential packet sticking.

        Args:
            timeout (float): Maximum time to wait for a complete response, in seconds.

        Returns:
            float: The parsed value from the response.

        Raises:
            TimeoutError: If the response is not received within the timeout period.
            ValueError: If the response is invalid or cannot be parsed.
        """
        import time

        start_time = time.time()
        response = bytearray()

        while time.time() - start_time < timeout:
            if self.serial.in_waiting > 0:
                byte = self.serial.read(1)  # Read one byte at a time
                response.extend(byte)

                # Check if we have received at least the minimum packet length
                if (
                    len(response) >= 8
                ):  # Address code (1) + Func (1) + Register address (2) + Command (2) + CRC (2)
                    if (
                        response[0] != 0x01  # Address code
                        or response[1] != 0x06  # Function code
                        or response[2] != 0x08  # Register address high byte
                        or response[3] != 0x00  # Register address low byte
                        or response[4] != 0xAB  # Command high byte
                        or response[5] != 0x56  # Command low byte
                        or response[6] != 0x74  # CRC high byte
                        or response[7] != 0xA4  # CRC low byte
                    ):
                        response.pop(0)  # Remove the first byte and continue
                        continue
                    return True

        raise TimeoutError(
            f"{self.name}: Response not received command response within {timeout} seconds."
        )


if __name__ == "__main__":
    import argparse
    import glob

    parser = argparse.ArgumentParser(description="Dial Indicator Test Program")
    parser.add_argument(
        "--port",
        type=str,
        help="The serial port for the dial indicator, e.g., /dev/ttyUSB0. If not specified, it will try to find one automatically.",
    )
    args = parser.parse_args()

    port_to_use = args.port

    if not port_to_use:
        available_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        if not available_ports:
            print(
                "Error: No serial ports found. Please specify one with the --port argument."
            )
            exit(1)
        print(f"Found available ports: {available_ports}")
        for port in available_ports:
            try:
                dial = DialIndicator(f"{port} Dial Indicator", port)
                dial.connect()
                try:
                    start_time = time.time()
                    while True:
                        value = dial.get_value()
                        print(f"{dial.name} value: {value}")
                        time.sleep(0.02)

                        if time.time() - start_time > 0.3:
                            break
                except KeyboardInterrupt:
                    print("Exiting...")
                    break
                except Exception as e:
                    print(f"Error with {dial.name}: {e}")
                finally:
                    dial.disconnect()
            except serial.SerialException:
                continue
    else:
        dial = DialIndicator("Dial Indicator", port_to_use)
        dial.connect()
        try:
            # success = dial.set_zero()
            # print(f"Set Zero Success: {success}")
            while True:
                value = dial.get_value()
                print(f"Dial Value: {value}")
                time.sleep(0.02)
        finally:
            dial.disconnect()