import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from msgs_clase.msg import Path   # type: ignore
from scipy import signal
import math
import numpy as np

class Path_generator(Node):
    def __init__(self):
        super().__init__('Path_generator')
        
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.declare_parameter('type', 2)
        
        self.pub = self.create_publisher(Path, 'path_generator', 1000)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Path generator node initialized')
        
        self.msg = Path()

    def timer_callback(self):
        ri_type = self.get_parameter('type').get_parameter_value().integer_value

        self.msg.type = ri_type

        if ri_type == 1:
            self.set_path_triangle()
            
        if ri_type == 2:
            self.set_path_square()

        if ri_type == 3:
            self.set_path_pentagon()
            
        if ri_type == 4:
            self.set_path_hexagon()

        self.apply_initial_offset()

        self.pub.publish(self.msg)

    def set_path_triangle(self):
        self.msg.x1 = 0.5
        self.msg.y1 = 0.87
        self.msg.x2 = 1.0
        self.msg.y2 = 0.0
        self.msg.x3 = 0.0
        self.msg.y3 = 0.0
        self.msg.x4 = 0.0
        self.msg.y4 = 0.0
        self.msg.x5 = 0.0
        self.msg.y5 = 0.0
        self.msg.x6 = 0.0
        self.msg.y6 = 0.0
        self.msg.x7 = 0.0
        self.msg.y7 = 0.0
        self.msg.x8 = 0.0
        self.msg.y8 = 0.0

    def set_path_square(self):
        self.msg.x1 = 1.0
        self.msg.y1 = 0.0
        self.msg.x2 = 1.0
        self.msg.y2 = 1.0
        self.msg.x3 = 0.0
        self.msg.y3 = 1.0
        self.msg.x4 = 0.0
        self.msg.y4 = 0.0
        self.msg.x5 = 0.0
        self.msg.y5 = 0.0
        self.msg.x6 = 0.0
        self.msg.y6 = 0.0
        self.msg.x7 = 0.0
        self.msg.y7 = 0.0
        self.msg.x8 = 0.0
        self.msg.y8 = 0.0

    def set_path_pentagon(self):
        self.msg.x1 = 0.35
        self.msg.y1 = 0.02
        self.msg.x2 = 0.9
        self.msg.y2 = 0.21
        self.msg.x3 = 0.9
        self.msg.y3 = 0.59
        self.msg.x4 = 0.35
        self.msg.y4 = 0.98
        self.msg.x5 = 0.0
        self.msg.y5 = 0.5
        self.msg.x6 = 0.35
        self.msg.y6 = 0.02
        self.msg.x7 = 0.0
        self.msg.y7 = 0.0
        self.msg.x8 = 0.0
        self.msg.y8 = 0.0

    def set_path_hexagon(self):
        self.msg.x1 = 0.25
        self.msg.y1 = 0.07
        self.msg.x2 = 0.75
        self.msg.y2 = 0.07
        self.msg.x3 = 1.0
        self.msg.y3 = 0.5
        self.msg.x4 = 0.75
        self.msg.y4 = 0.93
        self.msg.x5 = 0.25
        self.msg.y5 = 0.93
        self.msg.x6 = 0.0
        self.msg.y6 = 0.05
        self.msg.x7 = 0.25
        self.msg.y7 = 0.07
        self.msg.x8 = 0.0
        self.msg.y8 = 0.0

    def apply_initial_offset(self):
        dx = self.get_parameter('init_pose_x').value
        dy = self.get_parameter('init_pose_y').value
        for i in range(1, 9):
            setattr(self.msg, f'x{i}', getattr(self.msg, f'x{i}') + dx)
            setattr(self.msg, f'y{i}', getattr(self.msg, f'y{i}') + dy)
    


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Path_generator()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
