import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Vector, Path   # type: ignore
from geometry_msgs.msg import Twist
import math


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        # Se crea publicador para cmdvel
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)


        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Controller node initialized')

    def timer_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()