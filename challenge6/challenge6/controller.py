import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import transforms3d
from sensor_msgs.msg import LaserScan

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)

        self.initial_point_x = self.get_parameter('init_pose_x').value
        self.initial_point_y = self.get_parameter('init_pose_y').value

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Subscriptions
        self.subscription_odometry = self.create_subscription(
            Odometry,
            'odometria',
            self.callback_odometry,
            rclpy.qos.qos_profile_sensor_data )
        
        self.subscription_path = self.create_subscription(
            Goal,
            'path_generator',
            self.callback_path,
            rclpy.qos.qos_profile_sensor_data )
        
        # Variables para almacenar la posicoin actual del robot
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0

        # Lista para almacenar los puntos de trayectoria
        self.coordenadasMeta = []

        # Índice del punto actual en la trayectoria
        self.indice_punto_actual = 0

        # Velocidades lineal y angular del robot
        self.velL = 0.0
        self.velA = 0.0

        # Errores para el control
        self.error_distancia = 0.0
        self.angulo_objetivo = 0.0
        self.errorTheta = 0.0

        #Variables para el control
        #Theta 
        #Valores de k
        self.kpTheta = 0.8
        self.kiTheta = 0.0
        self.kdTheta = 0.0
        #Resultado operaciones
        self.PTheta = 0.0
        self.ITheta = 0.0
        self.DTheta = 0.0
        #Control
        self.UTheta = 0.0
        #Lineal
        #Valores de k
        self.kpLineal = 0.4
        self.kiLineal = 0.0
        self.kdLineal = 0.0
        #Resultados operaciones
        self.PLineal = 0.0
        self.ILineal = 0.0
        self.DLineal = 0.0
        #Control
        self.Ulineal = 0.0

        # Variables para distinguir trayectoria
        self.tipo_trayectoria_actual = 0
        self.tipo_trayectoria_prev = 0
        self.trayectoria_finalizda = False


        # Variables para bug0
        self.state = 'GO_TO_GOAL'

        self.subscription_lidar = self.create_subscription(LaserScan, 'scan', self.callback_lidar, 10)

    def timer_callback(self):
        
        # if self.waiting_new_trajectory():
        #     return
        
        if self.check_empty_trajectory():
            return

        if self.trayectoria_finalizda:
            self.velL = 0.0
            self.velA = 0.0
            # Se crea mensaje a publicar
            twist_msg = Twist()
            twist_msg.linear.x = self.velL
            twist_msg.angular.z = self.velA
            self.pub_cmd_vel.publish(twist_msg)
            self.get_logger().warn('Se ha encontrado el punto')
            return
        

        if self.state == 'GO_TO_GOAL':
            self.compute_errors()
            self.apply_control()
            self.limit_velocities()
            self.check_point_reached()
            self.publish_velocity_command()

        elif self.state == 'FOLLOW_WALL':
            # Comportamiento similar al de wall_follower.py pero adaptado aquí mismo
            self.wall_follow_logic()

    def is_path_clear_to_goal(self):
        front = self.get_range_from_lidar(0)
        return front > 0.6

    def get_range_from_lidar(self, angle_deg):
        if not hasattr(self, 'lidar_msg'):
            return float('inf')
        msg = self.lidar_msg
        ranges = np.array(self.latest_ranges)
        angle_rad = np.radians(angle_deg)
        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        if 0 <= index < len(ranges):
            val = ranges[index]
            return val if not (np.isnan(val) or np.isinf(val)) else float('inf')
        return float('inf')

    def callback_lidar(self, msg: LaserScan):
        self.latest_ranges = msg.ranges
        self.lidar_msg = msg

        ranges = np.array(msg.ranges)

        def get_range(angle_deg):
            angle_rad = np.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            if 0 <= index < len(ranges):
                value = ranges[index]
                return value if not (np.isnan(value) or np.isinf(value)) else float('inf')
            return float('inf')

        front = get_range(0)

        if self.state == 'GO_TO_GOAL' and front < 0.5:
            self.state = 'FOLLOW_WALL'
            self.get_logger().info('Cambio a modo FOLLOW_WALL')
        elif self.state == 'FOLLOW_WALL' and self.is_path_clear_to_goal():
            self.state = 'GO_TO_GOAL'
            self.get_logger().info('Regreso a modo GO_TO_GOAL')


        

    def callback_odometry(self, msg: Odometry):
        if msg is not None:
            self.Posx = msg.pose.pose.position.x
            self.Posy = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])  # Orden: w, x, y, z
            self.Postheta = yaw

    # Callback para recibir los puntos de la trayectoria
    def callback_path(self, msg):
        if msg is not None:
            self.coordenadasMeta = [msg.x_goal,msg.y_goal]
    
    def waiting_new_trajectory(self):
        if self.trayectoria_finalizda:
            if self.tipo_trayectoria_actual == self.tipo_trayectoria_prev:
                self.get_logger().warn('Esperado nueva trayectoria')
                return True
            else:
                self.trayectoria_finalizda = False
            return False

    def check_empty_trajectory(self):
        if not self.coordenadasMeta:
            self.get_logger().warn('No hay puntos en la trayectoria')
            self.velL = 0.0
            self.velA = 0.0
            # Se crea mensaje a publicar
            twist_msg = Twist()
            twist_msg.linear.x = self.velL
            twist_msg.angular.z = self.velA
            self.pub_cmd_vel.publish(twist_msg)
            return True
        return False
        
    def normalize_angle(self):
        if self.errorTheta >= math.pi:
            self.errorTheta -= 2 * math.pi
        elif self.errorTheta <= -math.pi:
            self.errorTheta += 2 * math.pi

    def compute_errors(self):
        # Coordenadas destino
        target_x = self.coordenadasMeta[0] * 1.1
        target_y = self.coordenadasMeta[1] * 1.1

        # Coordenadas previas
        target_x_ant = self.initial_point_x
        target_y_ant = self.initial_point_y

        # Calculo de coordenadas polares
        # Se calcula el error lineal
        self.error_distancia = math.sqrt((target_x - self.Posx)**2 + (target_y - self.Posy)**2)

        #Se calcula el error angular
        self.angulo_objetivo = math.atan2(target_y-target_y_ant, target_x-target_x_ant)
        self.errorTheta = self.angulo_objetivo - self.Postheta * 0.99
        self.normalize_angle()

    def apply_control(self):
        # Angular
        self.PTheta = self.kpTheta*self.errorTheta
        self.ITheta += self.timer_period*self.kiTheta*self.errorTheta
        self.DTheta = self.kdTheta/self.timer_period*self.errorTheta
        self.Ulineal = self.PTheta + self.ITheta + self.DTheta

        self.velA = self.kpTheta*self.errorTheta
        
        # Lineal
        self.velL = self.kpLineal*self.error_distancia

    def limit_velocities(self):
        if self.velA > 0.25:
            self.velA = 0.25

        if self.velL > 0.2:
            self.velL = 0.2

        if self.errorTheta > 0.05 or self.errorTheta < -0.05:
            self.velL = 0.0 

    def check_point_reached(self):
        if self.errorTheta < 0.05 and self.errorTheta > -0.05 and self.error_distancia < 0.05:
            self.trayectoria_finalizda = True


    def wall_follow_logic(self):
        # Parámetros
        desired_distance = 0.5  # distancia ideal a la pared (izquierda)
        max_linear = 0.2
        max_angular = 1.0
        kp = 1.0

        # Usa las últimas lecturas del lidar
        # Asegúrate de haber guardado las últimas ranges en el callback
        if not hasattr(self, 'latest_ranges') or not hasattr(self, 'lidar_msg'):
            self.get_logger().warn("Esperando datos de LiDAR...")
            return

        ranges = np.array(self.latest_ranges)
        msg = self.lidar_msg

        def get_range(angle_deg):
            angle_rad = np.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            if 0 <= index < len(ranges):
                val = ranges[index]
                return val if not (np.isnan(val) or np.isinf(val)) else float('inf')
            return float('inf')

        # Simulación de sensores virtuales
        front = get_range(0)
        left = get_range(90)
        left_corner = get_range(45)

        twist = Twist()

        if front < 0.4:
            # Giro a la derecha si hay pared de frente
            self.get_logger().info("Pared al frente. Giro a la derecha.")
            twist.linear.x = 0.0
            twist.angular.z = -max_angular
        elif left < desired_distance:
            if left_corner < desired_distance / 2:
                self.get_logger().info("Demasiado cerca de la pared izquierda. Corrección derecha.")
                twist.linear.x = max_linear
                twist.angular.z = -max_angular / 2
            else:
                self.get_logger().info("Siguiendo la pared.")
                twist.linear.x = max_linear
                twist.angular.z = 0.0
        else:
            self.get_logger().info("Pared lejos. Giro a la izquierda.")
            twist.linear.x = max_linear / 2
            twist.angular.z = max_angular / 2

        self.pub_cmd_vel.publish(twist)

    
    def publish_velocity_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.velL
        twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    rclpy.spin(m_t_p)
    m_t_p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()