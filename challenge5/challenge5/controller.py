import rclpy
from rclpy.node import Node
from msgs_clase.msg import Path   # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import transforms3d

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)

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
            Path,
            'path_generator',
            self.callback_path,
            rclpy.qos.qos_profile_sensor_data )
        
        # Variables para almacenar la posicoin actual del robot
        self.Posx = 0.0
        self.Posy = 0.0
        self.Postheta = 0.0

        # Lista para almacenar los puntos de trayectoria
        self.trayectoria = []

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
        self.trayectoria_finalizda = True

    def normalize_angle(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi

    def timer_callback(self):

        # Se verifica que se haya terminado la trayectoria y no existe una nueva
        
        if self.waiting_new_trajectory():
            return
        
        if self.check_empty_trajectory():
            return

        if self.check_trajectory_completed():
            return
        
        self.compute_errors()

        self.apply_control()

        self.limit_velocities()

        self.check_point_reached()

        self.update_trajectory_state()

        self.publish_velocity_command()
        

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
        if msg is not None and self.trayectoria_finalizda:
            self.trayectoria = [(self.initial_point_x,self.initial_point_y),
                                (msg.x1, msg.y1), 
                                (msg.x2, msg.y2),
                                (msg.x3, msg.y3), 
                                (msg.x4, msg.y4),
                                (msg.x5, msg.y5), 
                                (msg.x6, msg.y6),
                                (msg.x7, msg.y7),
                                (msg.x8, msg.y8)]
            self.tipo_trayectoria_actual = msg.type
    
    def waiting_new_trajectory(self):
        if self.trayectoria_finalizda:
            if self.tipo_trayectoria_actual == self.tipo_trayectoria_prev:
                self.get_logger().warn('Esperado nueva trayectoria')
                return True
            else:
                self.trayectoria_finalizda = False
            return False

    def check_empty_trajectory(self):
        if not self.trayectoria:
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

    def check_trajectory_completed(self):
        if self.trayectoria[self.indice_punto_actual] == (self.initial_point_x,self.initial_point_y) and self.indice_punto_actual != 0:
            self.get_logger().warn('Trayectoria terminada')
            self.trayectoria_finalizda = True
            self.velL = 0.0
            self.velA = 0.0
            # Se crea mensaje a publicar
            twist_msg = Twist()
            twist_msg.linear.x = self.velL
            twist_msg.angular.z = self.velA
            self.pub_cmd_vel.publish(twist_msg)
            self.indice_punto_actual = 0
            return True
        return False
    
    def normalize_angle(self):
        # De -pi a pi
        if self.errorTheta >= math.pi:
            self.errorTheta -= 2 * math.pi
        elif self.errorTheta <= -math.pi:
            self.errorTheta += 2 * math.pi


    def compute_errors(self):
        # Coordenadas destino
        target_x, target_y = self.trayectoria[self.indice_punto_actual+1]
        # Coordenadas previas
        target_x_ant, target_y_ant = self.trayectoria[self.indice_punto_actual]

        # Calculo de coordenadas polares
        # Se calcula el error lineal
        self.error_distancia = math.sqrt((target_x - self.Posx)**2 + (target_y - self.Posy)**2)

        #Se calcula el error angular
        self.angulo_objetivo = math.atan2(target_y-target_y_ant, target_x-target_x_ant)
        self.errorTheta = self.angulo_objetivo - self.Postheta
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
            self.indice_punto_actual += 1

    def update_trajectory_state(self):
        self.tipo_trayectoria_prev = self.tipo_trayectoria_actual
        self.trayectoria_finalizda = False
    
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