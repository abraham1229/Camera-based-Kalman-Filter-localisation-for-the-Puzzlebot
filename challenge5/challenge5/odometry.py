import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d
import math
import numpy as np

#Se crea el nodo My_Talker_Params tomando objeto de Node.
class Odometry_Node(Node):   

    #Se inicializa el nodo
    def __init__(self):
        #Se crear el nodo que será encargado de publicar la señal
        super().__init__('Odometria')

        # Declare the parameter with a default value
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', np.pi/2)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter("use_linear_model", True)

        #Se hacen las suscripciones pertinentes
        self.subscription_velocity_left = self.create_subscription(
            Float32,
            'VelocityEncL',
            self.signal_callback_left,
            rclpy.qos.qos_profile_sensor_data ) #Se debe de incluir la lectura de datos
        
        self.subscription_velocity_right = self.create_subscription(
            Float32,
            'VelocityEncR',
            self.signal_callback_right,
            rclpy.qos.qos_profile_sensor_data)
        
        #Se crea el publicador que mandará mensaje personalizado
        self.pub_odometry = self.create_publisher(Odometry, 'odometria', 1000)
        # Período de temporizador para 10Hz
        self.timer_period = 0.01 
        #Se declara el timer que llamará al callback
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        #DECLARACIÓN DE VARIABLES

        #Variables físicas del robot
        #Valor del radio de la llanta
        self.radius = 0.05
        #Valor de la distancia entre llantas
        self.lenght = 0.19

        #Variables de velocidad
        self.vel_left = 0.0 #Variable de lectura izquierda de la llanta
        self.vel_right = 0.0 #Variable de lectura derecha de la llanta
        
        #Obtención de velocidades
        self.velocidadTheta = 0.0 #Velocidad thetha
        self.velLineal = 0.0 #Variable para obtener las velocidades del robot

        self.Sigma = np.zeros((3, 3))

        self.covariance = 0.0001 * np.array([
            [0.1059, 0.1106, 0.1516],
            [0.1106, 1.7173, 0.6924],
            [0.1516, 0.6924, 1.5237]
        ])

        #Variables de odometría
        self.posX = self.get_parameter('init_pose_x').value
        self.posY = self.get_parameter('init_pose_y').value
        self.theta = self.get_parameter('init_pose_yaw').value
        self.use_linear_model = self.get_parameter("use_linear_model").get_parameter_value().bool_value

        #Se despliega en pantalla que se ha inicializado el nodo
        self.get_logger().info('Odometry node initialized')

        
    #Lee los datos del nodo de la llanta izquierda
    def signal_callback_left(self, msg):
        if msg is not None:
            self.vel_left = msg.data

    #Lee los datos del nodo de la llanta derecha
    def signal_callback_right(self, msg):
        if msg is not None:
            self.vel_right = msg.data

    # utils.py or top of your main file
    def normalize_angle(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    
    def linearized_state_update(self, s, u):
        # Linearization: Compute A_k and B_k
        A_k = np.array([
            [1, 0, -u[0] * math.sin(self.theta) * self.timer_period],
            [0, 1,  u[0] * math.cos(self.theta) * self.timer_period],
            [0, 0, 1]
        ])

        B_k = np.array([
            [math.cos(self.theta) * self.timer_period, 0],
            [math.sin(self.theta) * self.timer_period, 0],
            [0, self.timer_period]
        ])

        # State update using linearized model
        s_new = A_k @ s + B_k @ u

        # Propagate covariance using affine transform
        Sigma_new = A_k @ self.Sigma @ A_k.T + self.covariance

        return s_new, Sigma_new


    
    #Se hace callback en el que se calcula la posición en x, y y theta
    def timer_callback(self):

        #Se calcula theta punto
        self.velocidadTheta = self.radius*((self.vel_right-self.vel_left)/self.lenght)
        #Se calcula argumento velocidad
        self.velLineal = self.radius*((self.vel_right+self.vel_left)/2) * 1.09
        #Se calcula theta
        self.theta += self.velocidadTheta * self.timer_period * 1.1722
                
        #Se calcula posición en x y y
        self.posX += self.velLineal*math.cos(self.theta) *self.timer_period
        self.posY += self.velLineal*math.sin(self.theta) *self.timer_period

        # Create Odometry message
        odom_msg = Odometry()

        if self.use_linear_model:
            # Current state
            s = np.array([self.posX, self.posY, self.theta])
            u = np.array([self.velLineal, self.velocidadTheta])

            ## Call the linearized state update function
            s_new, self.Sigma = self.linearized_state_update(s, u)

            # Linearization: Compute A_k and B_k
            A_k = np.array([
                [1, 0, -self.velLineal * math.sin(self.theta) * self.timer_period],
                [0, 1,  self.velLineal * math.cos(self.theta) * self.timer_period],
                [0, 0, 1]
            ])

            B_k = np.array([
                [math.cos(self.theta) * self.timer_period, 0],
                [math.sin(self.theta) * self.timer_period, 0],
                [0, self.timer_period]
            ])

            # State update using linearized model
            s_new = A_k @ s + B_k @ u
            #self.posX, self.posY, self.theta = s_new

            # Propagate covariance using affine transform
            self.Sigma = A_k @ self.Sigma @ A_k.T + self.covariance

            # Update the covariance in the odometry message
            odom_msg.pose.covariance = [
            self.Sigma[0, 0], self.Sigma[0, 1], 0.0, 0.0, 0.0, self.Sigma[0, 2], # Row 1
            self.Sigma[1, 0], self.Sigma[1, 1], 0.0, 0.0, 0.0, self.Sigma[1, 2], # Row 2
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 3 (z)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 4 (roll)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                                        # Row 5 (pitch)
            self.Sigma[2, 0], self.Sigma[2, 1], 0.0, 0.0, 0.0, self.Sigma[2, 2]  # Row 6 (yaw)
            ]

        #Normalize theta
        self.theta = self.normalize_angle(self.theta)

        #Odom message
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value.strip('/')
        ns = self.get_namespace().strip('/')
        odom_msg.child_frame_id = f'{ns}/base_link' if ns else 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.posX
        odom_msg.pose.pose.position.y = self.posY
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]

        # TWIST
        odom_msg.twist.twist.linear.x  = self.velLineal # linear velocity (forward)
        odom_msg.twist.twist.angular.z = self.velocidadTheta # angular velocity (around Z)

        # Publish the Odometry message
        self.pub_odometry.publish(odom_msg)

        

#La función que será llamada según nuestro setup
def main(args=None):
    #Se inicializa el entorno de ros
    rclpy.init(args=args)
    #Se crea una instancia de la clase creada previamente.
    m_t_p = Odometry_Node()
    #Creación del bucle de eventos
    rclpy.spin(m_t_p)
    #Se liberan recursos
    m_t_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
