import time
import sys
import serial.tools.list_ports

# 确保能 import 到 components
from components.sensor.laser_indicator import LaserIndicator


BAUDRATE = 9600
SCAN_TIMEOUT = 0.5
READ_INTERVAL = 0.05   # 20 Hz


def scan_laser_ports():
    """
    扫描所有串口，找出可以正常返回激光数值的端口
    """
    laser_ports = []

    ports = list(serial.tools.list_ports.comports())
    print("🔍 Scanning serial ports...\n")

    for port in ports:
        print(f"Trying {port.device} ({port.description})")
        try:
            laser = LaserIndicator(f"LaserTest@{port.device}", port.device)
            laser.connect()
            time.sleep(0.1)

            value = laser.get_value()
            laser.disconnect()

            if value is not None:
                print(f"  ✅ Laser detected: {port.device}, value = {value:.3f} mm\n")
                laser_ports.append(port.device)
            else:
                print("  ❌ No valid data\n")

        except Exception as e:
            print(f"  ❌ Failed: {e}\n")

    return laser_ports


def main():
    laser_ports = scan_laser_ports()

    if len(laser_ports) != 2:
        print("❌ ERROR: Expected 2 laser sensors.")
        print(f"Found: {laser_ports}")
        sys.exit(1)

    left_port, right_port = laser_ports

    print("🎯 Laser configuration:")
    print(f"  Left  laser -> {left_port}")
    print(f"  Right laser -> {right_port}\n")

    laser_left = LaserIndicator("LaserLeft", left_port)
    laser_right = LaserIndicator("LaserRight", right_port)

    try:
        laser_left.connect()
        laser_right.connect()
        time.sleep(0.2)

        print("📡 Dual laser real-time monitoring started")
        print("Press Ctrl+C to stop\n")

        while True:
            t = time.time()

            left_val = laser_left.get_value()
            right_val = laser_right.get_value()

            print(
                f"[{t:.3f}] "
                f"Left: {left_val:8.3f} mm | "
                f"Right: {right_val:8.3f} mm"
            )

            time.sleep(READ_INTERVAL)

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")

    finally:
        laser_left.disconnect()
        laser_right.disconnect()
        print("🔌 Serial ports closed")


if __name__ == "__main__":
    main()
