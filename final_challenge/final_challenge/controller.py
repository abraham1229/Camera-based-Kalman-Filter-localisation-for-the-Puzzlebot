import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal          # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math
import numpy as np
import transforms3d
import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Parámetros de control PD
        self.kp_linear = 0.8
        self.kd_linear = 0.2
        self.kp_angular = 1.5
        self.kd_angular = 0.3

        # Estado
        self.goal = None
        self.Posx = self.Posy = self.Postheta = 0.0
        self.prev_error_dist = 0.0
        self.prev_error_theta = 0.0
        self.prev_Posx = self.prev_Posy = 0.0
        self.goal_idx = 0
        self.final_goal_reached = False
        self.last_goal_time = self.get_clock().now()

        # Bug2 state
        self.state = 'GO_TO_GOAL'  # GO_TO_GOAL | FOLLOW_WALL
        self.obstacle_threshold_enter = 0.5
        self.obstacle_threshold_exit  = 0.6
        self.last_follow_wall_time = 0.0
        self.mline_slope = None
        self.mline_intercept = None
        self.initial_point_x = 0.0
        self.initial_point_y = 0.0

        # Lidar
        self.lidar_msg = None
        self.latest_ranges = []

        # Publicador y suscriptores
        qos = rclpy.qos.qos_profile_sensor_data
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_next_goal = self.create_publisher(Bool, 'next_goal', 10)
        self.create_subscription(Odometry, 'odometria', self.callback_odometry, qos)
        self.create_subscription(Goal, 'path_generator', self.callback_goal, qos)
        self.create_subscription(LaserScan, 'scan', self.callback_lidar, qos)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Controller node initialized')

    def callback_odometry(self, msg: Odometry):
        if msg is None:
            return
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_goal(self, msg: Goal):
        if msg is None:
            return
        # Detectar mensaje especial de fin (inf)
        if math.isinf(msg.x_goal) or math.isinf(msg.y_goal):
            self.final_goal_reached = True
            self.print_success('¡Meta final alcanzada!')
            return
        self.goal = (msg.x_goal, msg.y_goal)
        # Guardar punto inicial para la M-line
        self.initial_point_x = self.Posx
        self.initial_point_y = self.Posy
        x1, y1 = self.initial_point_x, self.initial_point_y
        x2, y2 = self.goal
        if abs(x2 - x1) > 1e-6:
            self.mline_slope = (y2 - y1) / (x2 - x1)
            self.mline_intercept = y1 - self.mline_slope * x1
        else:
            self.mline_slope = float('inf')
            self.mline_intercept = x1
        self.state = 'GO_TO_GOAL'

    def callback_lidar(self, msg: LaserScan):
        self.lidar_msg = msg
        self.latest_ranges = msg.ranges

        def r(angle_deg: int) -> float:
            ang = math.radians(angle_deg)
            idx = int((ang - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(msg.ranges):
                v = msg.ranges[idx]
                return v if not (math.isnan(v) or math.isinf(v)) else float('inf')
            return float('inf')

        obstacle_in_front = any(r(a) < self.obstacle_threshold_enter for a in range(-5, 90))
        front_clear       = all(r(a) > self.obstacle_threshold_exit  for a in range(-5, 90))

        def is_on_mline():
            tolerance = 0.3
            if self.mline_slope is None:
                return False
            if self.mline_slope == float('inf'):
                return abs(self.Posx - self.mline_intercept) < tolerance
            elif self.mline_slope == 0:
                return abs(self.Posy - self.mline_intercept) < tolerance
            else:
                expected_y = self.mline_slope * self.Posx + self.mline_intercept
                return abs(self.Posy - expected_y) < tolerance

        # Transiciones de estado
        if self.state == 'GO_TO_GOAL':
            if obstacle_in_front:
                self.state = 'FOLLOW_WALL'
                self.last_follow_wall_time = time.time()
                self.get_logger().info('Cambio a FOLLOW_WALL (Bug2)')
        elif self.state == 'FOLLOW_WALL':
            cond_time = time.time() - self.last_follow_wall_time > 2.0
            if front_clear and is_on_mline() and cond_time:
                self.state = 'GO_TO_GOAL'
                self.get_logger().info('Regreso a GO_TO_GOAL (Bug2)')

    def timer_callback(self):
        if self.final_goal_reached or self.goal is None:
            self.stop_robot()
            return

        if self.state == 'GO_TO_GOAL':
            self.go_to_goal_pd()
        elif self.state == 'FOLLOW_WALL':
            self.basic_wall_follow_step()

    def go_to_goal_pd(self):
        gx, gy = self.goal
        dx = gx - self.Posx
        dy = gy - self.Posy
        error_dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        error_theta = (target_theta - self.Postheta + math.pi) % (2 * math.pi) - math.pi

        # Derivadas
        d_error_dist = (error_dist - self.prev_error_dist) / self.timer_period
        d_error_theta = (error_theta - self.prev_error_theta) / self.timer_period

        # Control PD
        v = self.kp_linear * error_dist + self.kd_linear * d_error_dist
        w = self.kp_angular * error_theta + self.kd_angular * d_error_theta

        # Limitar velocidades
        v = max(min(v, 0.3), -0.3)
        w = max(min(w, 1.0), -1.0)

        # --- Producto escalar para detectar cruce del objetivo ---
        dx_prev = gx - self.prev_Posx
        dy_prev = gy - self.prev_Posy
        dx_now = gx - self.Posx
        dy_now = gy - self.Posy
        dot_product = dx_prev * dx_now + dy_prev * dy_now

        now = self.get_clock().now()
        time_since_last_goal = (now - self.last_goal_time).nanoseconds / 1e9
        reached = False
        if error_dist < 0.05 and time_since_last_goal > 0.5:
            reached = True
        elif dot_product < 0 and time_since_last_goal > 0.5:
            reached = True
            self.get_logger().info('Objetivo cruzado por producto escalar')
        if reached:
            v = 0.0
            w = 0.0
            self.goal_idx += 1
            self.prev_error_dist = 0.0
            self.prev_error_theta = 0.0
            self.get_logger().info(f'Punto {self.goal_idx} alcanzado')
            # Publicar señal para avanzar al siguiente objetivo
            msg = Bool()
            msg.data = True
            self.pub_next_goal.publish(msg)
            self.goal = None  # Esperar siguiente objetivo
            self.last_goal_time = now

        # Publicar comando
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd_vel.publish(twist)

        # Guardar errores previos
        self.prev_error_dist = error_dist
        self.prev_error_theta = error_theta
        self.prev_Posx = self.Posx
        self.prev_Posy = self.Posy

    def basic_wall_follow_step(self):
        desired_distance = 0.7
        max_linear  = 0.2
        max_angular = 2.0

        msg = self.lidar_msg
        if msg is None or not self.latest_ranges:
            self.stop_robot()
            return
        ranges = np.array(self.latest_ranges)

        def r(angle_deg):
            ang = math.radians(angle_deg)
            idx = int((ang - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(ranges):
                v = ranges[idx]
                return v if not (math.isnan(v) or math.isinf(v)) else float('inf')
            return float('inf')

        front       = r(0)
        left        = np.mean([r(a) for a in range(85, 95)])
        left_corner = np.mean([r(a) for a in range(40, 45)])

        twist = Twist()
        if front < 0.8:
            twist.linear.x  = 0.0
            twist.angular.z = -max_angular
        elif left < desired_distance:
            if left_corner < desired_distance / 2:
                twist.linear.x  = max_linear
                twist.angular.z = -max_angular / 2
            else:
                twist.linear.x  = max_linear
                twist.angular.z = 0.0
        else:
            twist.linear.x  = max_linear / 2
            twist.angular.z = max_angular / 2

        self.pub_cmd_vel.publish(twist)

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())

    def print_success(self, msg):
        GREEN = '\033[92m'
        RESET = '\033[0m'
        self.get_logger().info(f'{GREEN}{msg}{RESET}')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
