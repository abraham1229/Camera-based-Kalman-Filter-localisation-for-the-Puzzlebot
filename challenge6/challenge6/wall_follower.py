import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class RightObjectSeeker(Node):
    def __init__(self):
        super().__init__('right_object_seeker')
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Right Object Seeker Node Started.")

        # Parámetros
        self.angle_range_deg = (-100, -80)  # Búsqueda en esta ventana
        self.distance_threshold = 1.0       # Umbral para considerar "objeto detectado"

    def scan_callback(self, msg: LaserScan):
        angles_deg = np.degrees(np.arange(msg.angle_min, msg.angle_max, msg.angle_increment))
        ranges = np.array(msg.ranges)

        # Filtrar ángulos entre -100° y -80°
        indices = np.where((angles_deg >= self.angle_range_deg[0]) & (angles_deg <= self.angle_range_deg[1]))[0]

        if len(indices) == 0:
            self.get_logger().warn("No hay datos en el rango buscado.")
            return

        region_ranges = ranges[indices]
        min_distance = np.nanmin(region_ranges)

        twist = Twist()

        if np.isfinite(min_distance) and min_distance < self.distance_threshold:
            self.get_logger().info(f"¡Objeto detectado a la derecha a {min_distance:.2f}m! Acercando...")
            twist.linear.x = 0.1
            twist.angular.z = -0.3  # gira a la derecha mientras se mueve
        else:
            self.get_logger().info("Buscando objeto a la derecha...")
            twist.linear.x = 0.0
            twist.angular.z = -0.2  # gira lentamente a la derecha

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RightObjectSeeker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
