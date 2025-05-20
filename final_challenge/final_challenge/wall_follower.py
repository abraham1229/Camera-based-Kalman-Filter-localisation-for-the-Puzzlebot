import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class WallFollowerLidar(Node):
    def __init__(self):
        super().__init__('wall_follower_lidar')

        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parámetros de control
        self.kp = 1.0
        self.wall_desired = 0.5      # distancia deseada a la pared derecha
        self.linear_velocity = 0.4
        self.max_angular = 2.0
        self.threshold_front = 0.6   # si algo está más cerca, se considera obstáculo

        self.get_logger().info("Wall Follower with LiDAR node started")

    def get_distance_at_angle(self, msg, angle_deg):
        angle_rad = np.radians(angle_deg) % (2 * np.pi)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        if 0 <= index < len(msg.ranges):
            value = msg.ranges[index]
            if not np.isnan(value) and not np.isinf(value):
                return value
        return 2.0

    def scan_callback(self, msg: LaserScan):
        dist_right = self.get_distance_at_angle(msg, -90)
        dist_right_45 = self.get_distance_at_angle(msg, -45)
        dist_right_side = np.mean([dist_right, dist_right_45])
        dist_front = self.get_distance_at_angle(msg, 0)
        dist_front_5 = self.get_distance_at_angle(msg, -15)
        dist_front_mean = np.mean([dist_front, dist_front_5])

        twist = Twist()

        if dist_front_mean > self.threshold_front:
            error = dist_right_side - self.wall_desired
            turn_rate = -error * self.kp
            twist.linear.x = self.linear_velocity
            twist.angular.z = turn_rate

            # Diagnóstico de giro
            self.get_logger().info(f"Error: {error}")
        else:
            # Obstáculo al frente, detener avance y girar a la izquierda
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular
            self.get_logger().info("Obstáculo al frente → Giro a la izquierda")

        # Publicar comando final
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
