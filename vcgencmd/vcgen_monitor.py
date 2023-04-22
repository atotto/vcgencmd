#!/usr/bin/env python3


import binascii
import rclpy
import re
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Temperature
import subprocess


class ThrottleBit:
    def __init__(self, offset, meaning):
        self.offset = offset
        self.meaning = meaning


class ThrottleBits:
    def __init__(self):

        # Could use ctypes, or a custom package, but unclear how to handle sparse flags
        self.flags = [
            ThrottleBit(0, "Under-voltage detected"),
            ThrottleBit(1, "Arm frequency capped"),
            ThrottleBit(2, "Currently throttled"),
            ThrottleBit(3, "Soft temperature limit active"),
            ThrottleBit(16, "Under-voltage has occurred"),
            ThrottleBit(17, "Arm frequency capping has occurred"),
            ThrottleBit(18, "Throttling has occurred"),
            ThrottleBit(19, "Soft temperature limit has occurred"),
        ]


def make_even_hexstr(hex_str):
    lhs, rhs = re.split(r"x|X", hex_str)
    if bool(len(rhs) % 2):
        return "".join(["0", rhs])
    return rhs


def get_serialno():
    cpuserial = ""
    with open("/proc/cpuinfo", "r") as f:
        for line in f:
            if line.startswith("Serial"):
                cpuserial = line.split(":")[1].strip()

    return cpuserial


class VcgencmdMonitor(Node):
    def __init__(self):
        super().__init__("vcgencmd_monitor")

        self.pub_temp = self.create_publisher(Temperature, "pi_temperature", 5)
        self.pub_diag = self.create_publisher(DiagnosticArray, "diagnostics", 5)
        self.period = 1.0
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.throttle_bits = ThrottleBits()
        self.serial_no = get_serialno()

    def build_pub_temperature(self):
        temp = Temperature()
        temp.header.stamp = self.get_clock().now().to_msg()
        temp.header.frame_id = "pi"

        output = subprocess.check_output(["vcgencmd", "measure_temp"]).decode("utf-8")
        v = float(output[output.find("=") + 1 :].strip().rstrip("'C"))
        temp.temperature = v
        self.get_logger().info(f"Temperature: {v}v")

        self.pub_temp.publish(temp)

    def build_throttled_diag(self):
        voltage_diag_healthy = DiagnosticStatus()
        voltage_diag_healthy.level = DiagnosticStatus.OK
        voltage_diag_healthy.name = "vcgen throttling"
        voltage_diag_healthy.message = "vcgen healthy stats"
        voltage_diag_healthy.hardware_id = self.serial_no
        voltage_diag_unhealthy = DiagnosticStatus()
        voltage_diag_unhealthy.level = DiagnosticStatus.WARN
        voltage_diag_unhealthy.name = "vcgen throttling"
        voltage_diag_unhealthy.message = "vcgen unhealthy stats"
        voltage_diag_unhealthy.hardware_id = self.serial_no

        # https://www.raspberrypi.com/documentation/computers/os.html#get_throttled
        output = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8")
        tout = output.split("=")[1].strip()

        throttle_state = int.from_bytes(
            binascii.unhexlify(make_even_hexstr(tout.strip())), byteorder="little"
        )

        for flag in self.throttle_bits.flags:
            flag_active = (throttle_state >> flag.offset) & 1
            if flag_active:
                voltage_diag_unhealthy.values.append(
                    KeyValue(key=flag.meaning, value="Active")
                )
                self.get_logger().warning(f"{flag.meaning}")

            else:
                voltage_diag_healthy.values.append(
                    KeyValue(key=flag.meaning, value="Inactive")
                )

        return [voltage_diag_healthy, voltage_diag_unhealthy]

    def build_pub_diagnostics(self):
        diag = DiagnosticArray()
        diag.status.extend(self.build_throttled_diag())
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.header.frame_id = "pi"

        self.pub_diag.publish(diag)

    def timer_callback(self):
        self.build_pub_temperature()
        self.build_pub_diagnostics()


def main(args=None):
    rclpy.init(args=args)
    sensor = VcgencmdMonitor()
    rclpy.spin(sensor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
