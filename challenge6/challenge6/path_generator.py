import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
import numpy as np

class Path_generator(Node):
    def __init__(self):
        super().__init__('Path_generator')
        
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.declare_parameter('x_goal', 1.0)
        self.declare_parameter('y_goal', 1.0)
        
        self.pub = self.create_publisher(Goal, 'path_generator', 1000)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Path generator node initialized')
        
        self.msg = Goal()

    def timer_callback(self):
        self.msg.x_goal = self.get_parameter('x_goal').value
        self.msg.y_goal = self.get_parameter('y_goal').value
        self.apply_initial_offset()
        self.pub.publish(self.msg)
    
    def apply_initial_offset(self):
        self.msg.x_goal += self.get_parameter('init_pose_x').value
        self.msg.y_goal += self.get_parameter('init_pose_y').value
    


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Path_generator()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
