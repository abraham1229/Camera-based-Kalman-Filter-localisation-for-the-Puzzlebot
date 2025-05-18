import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
from std_msgs.msg import Bool
import numpy as np

class Path_generator(Node):
    def __init__(self):
        super().__init__('Path_generator')
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.num_goals = self.declare_parameter('num_goals', 3).value  # Número de puntos a enviar
        # Declarar parámetros para cada punto
        for i in range(1, self.num_goals+1):
            self.declare_parameter(f'x_goal_{i}', float(i))
            self.declare_parameter(f'y_goal_{i}', float(i))
        self.pub = self.create_publisher(Goal, 'path_generator', 10)
        self.goal_list = []
        self.current_goal_idx = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Path generator node initialized')
        self.init_offset_x = self.get_parameter('init_pose_x').value
        self.init_offset_y = self.get_parameter('init_pose_y').value
        self.finished = False
        self.next_goal_flag = False
        self.create_subscription(Bool, 'next_goal', self.next_goal_callback, 10)
        self.prepare_goal_list()

    def prepare_goal_list(self):
        self.goal_list = []
        for i in range(1, self.num_goals+1):
            x = self.get_parameter(f'x_goal_{i}').value + self.init_offset_x
            y = self.get_parameter(f'y_goal_{i}').value + self.init_offset_y
            goal = Goal()
            goal.x_goal = x
            goal.y_goal = y
            self.goal_list.append(goal)
        self.current_goal_idx = 0

    def next_goal_callback(self, msg: Bool):
        if msg.data and not self.finished:
            self.next_goal_flag = True

    def timer_callback(self):
        if not self.goal_list or self.finished:
            return
        # Publica el objetivo actual
        self.pub.publish(self.goal_list[self.current_goal_idx])
        # Cambia de objetivo solo si recibió la señal next_goal
        if self.next_goal_flag:
            self.next_goal_flag = False
            if self.current_goal_idx < len(self.goal_list) - 1:
                self.current_goal_idx += 1
                self.get_logger().info(f'Enviando objetivo {self.current_goal_idx + 1} de {len(self.goal_list)}')
            else:
                # Enviar un mensaje especial con NaN para indicar que terminó
                nan_goal = Goal()
                nan_goal.x_goal = float('inf')
                nan_goal.y_goal = float('inf')
                self.pub.publish(nan_goal)
                self.print_success('¡Todos los objetivos han sido enviados!')
                self.finished = True


    def print_success(self, msg):
        GREEN = '\033[92m'
        RESET = '\033[0m'
        self.get_logger().info(f'{GREEN}{msg}{RESET}')

def main(args=None):
    rclpy.init(args=args)
    m_t_p = Path_generator()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
