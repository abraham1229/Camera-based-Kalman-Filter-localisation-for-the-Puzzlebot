import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal          # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import math
import numpy as np
import transforms3d
import time


class Controller(Node):
    # --------------------------------------------------------------------- #
    #                           INICIALIZACIÓN                              #
    # --------------------------------------------------------------------- #
    def __init__(self):
        super().__init__('Controller')

        # --------- parámetros de lanzamiento --------------------------------
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('algorithm',  'bug2')   # 'bug0'  o  'bug2'

        self.initial_point_x = self.get_parameter('init_pose_x').value
        self.initial_point_y = self.get_parameter('init_pose_y').value
        self.algorithm       = self.get_parameter('algorithm').value

        # --------- editores / temporizador -----------------------------------
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer       = self.create_timer(0.1, self.timer_callback)

        # --------- suscripciones --------------------------------------------
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Odometry,  'odometria',      self.callback_odometry, qos)
        self.create_subscription(Goal,      'path_generator', self.callback_path,     qos)
        self.create_subscription(LaserScan, 'scan',           self.callback_lidar,    10)

        # --------- variables de estado ---------------------------------------
        self.Posx = self.Posy = self.Postheta = 0.0
        self.coordenadasMeta: list[float] = []

        self.kpTheta  = 0.8
        self.kpLineal = 0.4
        self.velL = self.velA = 0.0
        self.error_distancia = self.errorTheta = 0.0

        self.trayectoria_finalizda = False
        self.state = 'GO_TO_GOAL'                     # GO_TO_GOAL | FOLLOW_WALL

        # Umbrales para entrar / salir de FOLLOW_WALL
        self.obstacle_threshold_enter = 0.5
        self.obstacle_threshold_exit  = 0.6

        # --------- variables específicas Bug-2 -------------------------------
        self.mline_slope      = None
        self.mline_intercept  = None
        self.last_follow_wall_time = 0.0   # instante en el que entramos en FOLLOW_WALL


    # --------------------------------------------------------------------- #
    #                           CALLBACKS                                   #
    # --------------------------------------------------------------------- #
    def callback_odometry(self, msg: Odometry):
        self.Posx = msg.pose.pose.position.x
        self.Posy = msg.pose.pose.position.y
        qx, qy, qz, qw = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.Postheta = yaw

    def callback_path(self, msg: Goal):
        # Nueva meta
        self.coordenadasMeta = [msg.x_goal, msg.y_goal]

        # --- recalcular la M-line para Bug-2 ---
        if self.algorithm == 'bug2':
            x1, y1 = self.initial_point_x, self.initial_point_y
            x2, y2 = self.coordenadasMeta
            if abs(x2 - x1) > 1e-6:                        # pendiente finita
                self.mline_slope     = (y2 - y1) / (x2 - x1)
                self.mline_intercept = y1 - self.mline_slope * x1
            else:                                          # línea vertical
                self.mline_slope     = float('inf')
                self.mline_intercept = x1

    def callback_lidar(self, msg: LaserScan):
        self.lidar_msg     = msg
        self.latest_ranges = msg.ranges

        # Función local de rango
        def r(angle_deg: int) -> float:
            ang = math.radians(angle_deg)
            idx = int((ang - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(msg.ranges):
                v = msg.ranges[idx]
                return v if not (math.isnan(v) or math.isinf(v)) else float('inf')
            return float('inf')

        obstacle_in_front = any(r(a) < self.obstacle_threshold_enter for a in range(-5, 90))
        front_clear       = all(r(a) > self.obstacle_threshold_exit  for a in range(-5, 90))

        # ------------ ¿estamos sobre M-line? (para bug2) --------------------
        def on_mline() -> bool:
            if self.mline_slope is None:
                return False
            if self.mline_slope == float('inf'):
                return abs(self.Posx - self.mline_intercept) < 0.1
            expected_y = self.mline_slope * self.Posx + self.mline_intercept
            return abs(self.Posy - expected_y) < 0.1

        # Verificar si el robot está en la línea M-line con un rango de tolerancia
        def is_on_mline():
            tolerance = 0.2  # Rango de tolerancia para considerar que está en la línea imaginaria
            if self.mline_slope == float('inf'):
                return abs(self.Posx - self.mline_intercept) < tolerance
            expected_y = self.mline_slope * self.Posx + self.mline_intercept
            return abs(self.Posy - expected_y) < tolerance

        # ------------ transiciones de estado --------------------------------
        if self.state == 'GO_TO_GOAL':
            if obstacle_in_front:
                self.state = 'FOLLOW_WALL'
                self.last_follow_wall_time = time.time()
                self.get_logger().info(f'Cambio a FOLLOW_WALL ({self.algorithm})')

        elif self.state == 'FOLLOW_WALL':
            if self.algorithm == 'bug0':
                if front_clear and time.time() - self.last_follow_wall_time > 1.0:
                    self.state = 'GO_TO_GOAL'
                    self.get_logger().info('Regreso a GO_TO_GOAL (Bug0)')
            else:  # bug2
                cond_time = time.time() - self.last_follow_wall_time > 2.0   # <- 2 s
                if front_clear and is_on_mline() and cond_time:
                    self.state = 'GO_TO_GOAL'
                    self.get_logger().info('Regreso a GO_TO_GOAL (Bug2)')


    # --------------------------------------------------------------------- #
    #                          LÓGICA PRINCIPAL                             #
    # --------------------------------------------------------------------- #
    def timer_callback(self):
        if not self.coordenadasMeta:
            return

        if self.trayectoria_finalizda:
            self.stop_robot()
            return

        if self.state == 'GO_TO_GOAL':
            self.compute_errors()
            self.apply_control()
            self.limit_velocities()
            self.check_point_reached()
            self.publish_velocity_command()

        elif self.state == 'FOLLOW_WALL':
            if self.algorithm == 'bug0':
                self.basic_wall_follow_step()
            else:  # bug2
                self.bug2_logic()


    # --------------------------------------------------------------------- #
    #                       FUNCIONES DE CONTROL                            #
    # --------------------------------------------------------------------- #
    def compute_errors(self):
        gx, gy = self.coordenadasMeta
        dx, dy = gx - self.Posx, gy - self.Posy
        self.error_distancia = math.hypot(dx, dy)

        target_theta = math.atan2(dy, dx)
        self.errorTheta = (target_theta - self.Postheta + math.pi) % (2 * math.pi) - math.pi
        if abs(self.errorTheta) < 0.01:      # evitar pequeñas oscilaciones
            self.errorTheta = 0.0

    def apply_control(self):
        self.velA = self.kpTheta  * self.errorTheta
        self.velL = self.kpLineal * self.error_distancia

    def limit_velocities(self):
        self.velA = max(min(self.velA,  0.25), -0.25)
        self.velL = 0.0 if abs(self.errorTheta) > 0.1 else min(self.velL, 0.2)

    def check_point_reached(self):
        if abs(self.errorTheta) < 0.05 and self.error_distancia < 0.05:
            self.trayectoria_finalizda = True

    def publish_velocity_command(self):
        twist = Twist()
        twist.linear.x  = self.velL
        twist.angular.z = self.velA
        self.pub_cmd_vel.publish(twist)

    def stop_robot(self):
        self.pub_cmd_vel.publish(Twist())
        self.get_logger().warn('Meta alcanzada')


    # --------------------------------------------------------------------- #
    #                    PASO BÁSICO DE SEGUIMIENTO DE PARED                #
    # --------------------------------------------------------------------- #
    def basic_wall_follow_step(self):
        desired_distance = 0.7
        max_linear  = 0.2
        
        if self.algorithm == 'bug0':
            max_angular = 1.5
        else:  # bug2
             max_angular = 2.0

        msg    = self.lidar_msg
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


    # --------------------------------------------------------------------- #
    #                    LÓGICA COMPLETA BUG-2 (con retardo)                #
    # --------------------------------------------------------------------- #
    def bug2_logic(self):
        # --------- mantener 2 s siguiendo la pared antes de comprobar línea --
        if time.time() - self.last_follow_wall_time < 2.0:
            self.basic_wall_follow_step()
            return

        # ---------- una vez pasados 2 s, aplicar lógica Bug-2 ----------------
        desired_distance = 0.7
        max_linear  = 0.2
        max_angular = 1.0

        msg    = self.lidar_msg
        ranges = np.array(self.latest_ranges)

        def r(angle_deg):
            ang = math.radians(angle_deg)
            idx = int((ang - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(ranges):
                v = ranges[idx]
                return v if not (math.isnan(v) or math.isinf(v)) else float('inf')
            return float('inf')

        # -------- comprobar si objetivo está visible sobre la M-line ----------
        def on_mline():
            if self.mline_slope is None:
                return False
            if self.mline_slope == float('inf'):
                return abs(self.Posx - self.mline_intercept) < 0.1
            expected_y = self.mline_slope * self.Posx + self.mline_intercept
            return abs(self.Posy - expected_y) < 0.1

        # Verificar si el robot está en la línea M-line con un rango de tolerancia
        def is_on_mline():
            tolerance = 0.2  # Rango de tolerancia para considerar que está en la línea imaginaria
            if self.mline_slope == float('inf'):
                return abs(self.Posx - self.mline_intercept) < tolerance
            expected_y = self.mline_slope * self.Posx + self.mline_intercept
            return abs(self.Posy - expected_y) < tolerance

        goal_angle_deg = math.degrees(
            math.atan2(self.coordenadasMeta[1] - self.Posy,
                       self.coordenadasMeta[0] - self.Posx))
        goal_clear = r(goal_angle_deg) > self.obstacle_threshold_exit

        if goal_clear and is_on_mline():
            self.state = 'GO_TO_GOAL'
            self.get_logger().info('Objetivo visible sobre M-line → GO_TO_GOAL')
            return

        # -------- si no, seguir pared como siempre ---------------------------
        self.basic_wall_follow_step()


# ------------------------------------------------------------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
