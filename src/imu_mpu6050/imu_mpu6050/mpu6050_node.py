import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

from .mpu6050 import MPU6050


ACCEL_SCALE = 16384.0      # ±2g
GYRO_SCALE = 131.0         # ±250 deg/s


class MPU6050Node(Node):

    def __init__(self):

        super().__init__("mpu6050_node")

        self.publisher = self.create_publisher(
            Imu,
            "/imu/data_raw",
            10
        )

        self.sensor = MPU6050(bus_num=0)

        self.declare_parameter("publish_rate", 50.0)

        rate = self.get_parameter(
            "publish_rate"
        ).value

        self.timer = self.create_timer(
            1.0 / rate,
            self.publish_imu
        )

        self.get_logger().info("MPU6050 started")

    def publish_imu(self):

        ax, ay, az = self.sensor.read_accel_raw()
        gx, gy, gz = self.sensor.read_gyro_raw()

        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.linear_acceleration.x = ax / ACCEL_SCALE * 9.80665
        msg.linear_acceleration.y = ay / ACCEL_SCALE * 9.80665
        msg.linear_acceleration.z = az / ACCEL_SCALE * 9.80665

        msg.angular_velocity.x = math.radians(gx / GYRO_SCALE)
        msg.angular_velocity.y = math.radians(gy / GYRO_SCALE)
        msg.angular_velocity.z = math.radians(gz / GYRO_SCALE)

        # orientation unavailable
        msg.orientation_covariance[0] = -1.0

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = MPU6050Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping MPU6050 node")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
