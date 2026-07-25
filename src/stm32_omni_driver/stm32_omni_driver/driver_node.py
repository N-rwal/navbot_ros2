#!/usr/bin/env python3

import serial
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class STM32OmniDriver(Node):

    def __init__(self):
        super().__init__('stm32_omni_driver')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.declare_parameter('max_pwm', 255)

        self.declare_parameter('forward_gain', 1.0)
        self.declare_parameter('strafe_gain', 1.0)
        self.declare_parameter('rotation_gain', 1.0)

        self.declare_parameter('cmd_timeout', 0.5)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.max_pwm = self.get_parameter('max_pwm').value

        self.forward_gain = self.get_parameter('forward_gain').value
        self.strafe_gain = self.get_parameter('strafe_gain').value
        self.rotation_gain = self.get_parameter('rotation_gain').value

        self.timeout = self.get_parameter('cmd_timeout').value

        self.serial = serial.Serial(port, baud, timeout=1)

        self.last_cmd_time = self.get_clock().now()

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        self.timer = self.create_timer(
            0.1,
            self.timeout_check)

        self.get_logger().info(
            f"Connected to STM32 on {port}"
        )

    def wheel_mix(self, forward, rotation, strafe):

        fl = forward - rotation + strafe
        fr = forward + rotation - strafe
        bl = forward - rotation - strafe
        br = forward + rotation + strafe

        wheels = [fl, fr, bl, br]

        max_mag = max(abs(w) for w in wheels)

        if max_mag > 1.0:
            wheels = [w / max_mag for w in wheels]

        duties = [0] * 8

        mapping = [
            (6, 7),  # FL
            (4, 5),  # FR
            (2, 3),  # BL
            (0, 1)   # BR
        ]

        for wheel, (fwd_ch, rev_ch) in zip(wheels, mapping):

            pwm = int(min(abs(wheel) * self.max_pwm,
                          self.max_pwm))

            if wheel > 0.05:
                duties[fwd_ch] = pwm

            elif wheel < -0.05:
                duties[rev_ch] = pwm

        return duties

    def send(self, duties):
        try:
            self.serial.write(bytes(duties))
        except Exception as e:
            self.get_logger().error(str(e))

    def cmd_callback(self, msg):

        self.last_cmd_time = self.get_clock().now()

        forward = msg.linear.x * self.forward_gain
        strafe = -msg.linear.y * self.strafe_gain
        rotation = msg.angular.z * self.rotation_gain

        duties = self.wheel_mix(
            forward,
            rotation,
            strafe
        )

        self.send(duties)

    def timeout_check(self):

        elapsed = (
            self.get_clock().now()
            - self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout:
            self.send([0] * 8)


def main(args=None):

    rclpy.init(args=args)

    node = STM32OmniDriver()

    try:
        rclpy.spin(node)

    finally:
        node.send([0] * 8)
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
