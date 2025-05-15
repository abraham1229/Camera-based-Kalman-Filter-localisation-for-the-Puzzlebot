import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class WallFollowerLidar(Node):
    def __init__(self):
        super().__init__('wall_follower_lidar')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.wall_threshold = 0.5  # metros
        self.get_logger().info("Wall Follower with LiDAR node started")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)

        def get_range(angle_deg):
            angle_rad = np.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            if 0 <= index < len(ranges):
                value = ranges[index]
                return value if not (np.isnan(value) or np.isinf(value)) else float('inf')
            return float('inf')

        # Sensores simulados
        front = get_range(0)
        left = get_range(90)
        left_corner = get_range(45)

        # Comportamiento tipo Webots
        twist = Twist()
        max_linear = 0.3
        max_angular = 1.0

        if front < self.wall_threshold:
            self.get_logger().info("Pared al frente. Giro a la derecha.")
            twist.linear.x = 0.0
            twist.angular.z = -max_angular
        elif left < self.wall_threshold:
            if left_corner < self.wall_threshold / 2:
                self.get_logger().info("Muy cerca de la pared izquierda. Corrigiendo derecha.")
                twist.linear.x = max_linear
                twist.angular.z = -max_angular / 2
            else:
                self.get_logger().info("Pared a la izquierda. Avanzando recto.")
                twist.linear.x = max_linear
                twist.angular.z = 0.0
        else:
            self.get_logger().info("Sin pared. Giro a la izquierda.")
            twist.linear.x = max_linear / 2
            twist.angular.z = max_angular / 2

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
