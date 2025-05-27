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
        self.wall_desired = 0.3      # distancia deseada a la pared derecha
        self.max_linear = 0.1
        self.max_angular = 0.2
        self.threshold_front = 0.4   # si algo está más cerca, se considera obstáculo

        self.get_logger().info("Wall Follower with LiDAR node started")

    def get_distance_at_angle(self, msg: LaserScan, angle_deg: float):
        if msg is None:
            return 2.0

        # Convertir de grados a radianes
        angle_rad = np.radians(angle_deg)

        # Validar que el ángulo deseado está dentro del rango del LiDAR
        if not (msg.angle_min <= angle_rad <= msg.angle_max):
            return 2.0

        # Calcular el índice correspondiente en el arreglo
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)

        # Verificar y devolver la distancia si es válida
        if 0 <= index < len(msg.ranges):
            value = msg.ranges[index]
            if not np.isnan(value) and not np.isinf(value):
                return value

        return 2.0  # Valor por defecto si el índice no es válido



    def scan_callback(self, msg: LaserScan):
        dist_right = self.get_distance_at_angle(msg, -90)
        dist_right_45 = self.get_distance_at_angle(msg, -45)
        dist_right_side = min([dist_right, dist_right_45])
        dist_front = self.get_distance_at_angle(msg, 0)
        dist_front_5 = self.get_distance_at_angle(msg, -15)
        dist_front_mean = min([dist_front, dist_front_5])

        self.get_logger().info(f"Distancia frente: {dist_front_mean:.2f}")
        self.get_logger().info(f"Distancia right: {dist_right_side:.2f}")

        twist = Twist()

        if dist_front_mean > self.threshold_front:
            error = dist_right_side - self.wall_desired
            turn_rate = -error * self.kp
            twist.linear.x = self.max_linear
            twist.angular.z = turn_rate

            if abs(error) > 0.9: #to make sure it will reach the point
                twist.linear.x = self.max_linear / 5 

            # Diagnóstico de giro
            self.get_logger().info(f"Error: {error}")
        else:
            # Obstáculo al frente, detener avance y girar a la izquierda
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular
            # self.get_logger().info("Frente")

    
        
        twist.linear.x = max(min(twist.linear.x, self.max_linear), -self.max_linear)
        twist.angular.z = max(min(twist.angular.z, self.max_angular), -self.max_angular)

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
