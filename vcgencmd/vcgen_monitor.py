#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import subprocess


class VcgencmdMonitor(Node):
    def __init__(self):
        super().__init__("vcgencmd_monitor")

        self.pub = self.create_publisher(Temperature, "pi_temperature", 5)
        self.period = 1.0
        self.timer = self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        temp = Temperature()
        temp.header.stamp = self.get_clock().now().to_msg()
        temp.header.frame_id = "pi"

        output = subprocess.check_output(["vcgencmd", "measure_temp"]).decode("utf-8")
        v = float(output[output.find("=") + 1 :].strip().rstrip("'C"))
        temp.temperature = v
        self.get_logger().info(f"Temperature: {v}v")

        self.pub.publish(temp)


def main(args=None):
    rclpy.init(args=args)
    sensor = VcgencmdMonitor()
    rclpy.spin(sensor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
