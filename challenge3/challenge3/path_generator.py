import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Path   # type: ignore
from scipy import signal
import math


class My_Talker_Params(Node):
    def __init__(self):
        super().__init__('Path_generator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type', rclpy.Parameter.Type.INTEGER),
            ])
        
        self.pub = self.create_publisher(Path, 'path_generator', 1000)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Path generator node initialized')
        self.msg = Path()

    def timer_callback(self):
        ri_type = self.get_parameter('type').get_parameter_value().integer_value
 
        if ri_type == 0:
            # Square condition
            self.msg.x1 = 1.0
            self.msg.y1 = 1.0
            self.msg.x2 = 1.0
            self.msg.y2 = 1.0
            self.msg.x3 = 1.0
            self.msg.y3 = 1.0
            self.msg.x4 = 1.0
            self.msg.y4 = 1.0
        
        if ri_type == 1:
            self.msg.x1 = 2.0
            self.msg.y1 = 2.0
            self.msg.x2 = 2.0
            self.msg.y2 = 2.0
            self.msg.x3 = 2.0
            self.msg.y3 = 2.0
            self.msg.x4 = 2.0
            self.msg.y4 = 2.0
        

        self.pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = My_Talker_Params()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
