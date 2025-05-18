import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal          # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import transforms3d

class SimplePDController(Node):
    def __init__(self):
        super().__init__('simple_pd_controller')

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

        # Publicador y suscriptores
        qos = rclpy.qos.qos_profile_sensor_data
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', qos)
        self.create_subscription(Odometry, 'odometria', self.callback_odometry, qos)
        self.create_subscription(Goal, 'path_generator', self.callback_goal, qos)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Controller node initialized')

    def callback_odometry(self, msg: Odometry):
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_goal(self, msg: Goal):
        self.goal = (msg.x_goal, msg.y_goal)
        self.prev_error_dist = 0.0
        self.prev_error_theta = 0.0

    def timer_callback(self):
        if self.goal is None:
            return
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

        # Detener si está cerca del objetivo
        if error_dist < 0.05:
            v = 0.0
            w = 0.0
            self.get_logger().info('Meta alcanzada')

        # Publicar comando
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd_vel.publish(twist)

        # Guardar errores previos
        self.prev_error_dist = error_dist
        self.prev_error_theta = error_theta

def main(args=None):
    rclpy.init(args=args)
    node = SimplePDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
