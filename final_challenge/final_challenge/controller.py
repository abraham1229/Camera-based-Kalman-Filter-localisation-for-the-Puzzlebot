import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal          # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import transforms3d

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

        # Publicador y suscriptores
        qos = rclpy.qos.qos_profile_sensor_data
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_next_goal = self.create_publisher(Bool, 'next_goal', 10)
        self.create_subscription(Odometry, 'odometria', self.callback_odometry, qos)
        self.create_subscription(Goal, 'path_generator', self.callback_goal, qos)
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
            self.get_logger().warn('¡Meta final alcanzada!')
            return
        self.goal = (msg.x_goal, msg.y_goal)

    def timer_callback(self):
        if self.final_goal_reached or self.goal is None:
            return
        gx, gy = self.goal
        dx = gx - self.Posx
        dy = gy - self.Posy
        error_dist = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        error_theta = (target_theta - self.Postheta + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().info(f'Error lineal {self.prev_error_dist}')
        self.get_logger().info(f'Error angular {self.prev_error_theta}')

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

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
