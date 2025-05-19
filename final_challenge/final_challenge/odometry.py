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
        self.declare_parameter('init_pose_yaw', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter("use_linear_model", True)
        self.declare_parameter('motion_model_noise', False)

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
        # Initialize Sigma (avoid singularities)
        self.Sigma = 1e-3 * np.zeros((3, 3))

        self.covariance = 0.0001 * np.array([
            [0.1059, 0.1106, 0.1516],
            [0.1106, 1.7173, 0.6924],
            [0.1516, 0.6924, 1.5237]
        ])

        self.R = np.array([
            [0.1, 0],
            [0, 0.1]
        ])
        # Variables for Kalman filter
        self.have_new_aruco = False
        #linear_v -> lv, lv -> rotation, angular_v -> l_motion, av -> rotation, l_noise -> a_nosie
        self.alpha = [0.05, 0.001, 0.05, 0.01, 0.01]
        # NIS monitoring
        self.nis_values = []
        self.nis_window_size = 30
        self.chi2_lower = 0.103  # chi2 lower bound for 95% confidence, 2 DoF
        self.chi2_upper = 5.991  # chi2 upper bound for 95% confidence, 2 DoF

        #Variables de odometría
        self.posX = self.get_parameter('init_pose_x').value
        self.posY = self.get_parameter('init_pose_y').value
        self.theta = self.get_parameter('init_pose_yaw').value
        self.use_linear_model = self.get_parameter('use_linear_model').get_parameter_value().bool_value
        self.motion_noise = self.get_parameter('motion_model_noise').get_parameter_value().bool_value
        #Se despliega en pantalla que se ha inicializado el nodo
        self.get_logger().info('Odometry node initialized')

        
    #Se hace callback en el que se calcula la posición en x, y y theta
    def timer_callback(self):

        #Se calcula theta punto
        self.velocidadTheta = self.radius*((self.vel_right-self.vel_left)/self.lenght)
        #Se calcula argumento velocidad
        self.velLineal = self.radius*((self.vel_right+self.vel_left)/2) * 1.09
        #Se calcula theta
        self.theta += self.velocidadTheta * self.timer_period
                
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
            self.s, self.Sigma = self.linearized_state_update(s, u)
            if self.have_new_aruco:
                z = self.latest_aruco_measurement  # [range, bearing]
                landmark = self.latest_landmark_pos # [x, y]
                self.kalman_filter(z, landmark)
                self.have_new_aruco = False

            #Update Robot position
            self.posX, self.posY, self.theta = self.s
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

    #Lee los datos del nodo de la llanta izquierda
    def signal_callback_left(self, msg):
        if msg is not None:
            self.vel_left = msg.data if abs(msg.data) > 1e-3 else 0.0

    #Lee los datos del nodo de la llanta derecha
    def signal_callback_right(self, msg):
        if msg is not None:
            self.vel_right = msg.data if abs(msg.data) > 1e-3 else 0.0

    # utils.py or top of your main file
    def normalize_angle(self, theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    
    def noise(self, v, w):
        # Linear velocity variance depends on both velocities
        var_v = (self.alpha[0] * v**2) + (self.alpha[1] * w**2)
        # Angular velocity variance depends on both velocities
        var_w = (self.alpha[2] * v**2) + (self.alpha[3] * w**2)
        # Cross-correlation term
        var_vw = self.alpha[4] * v * w  

        return np.array([
            [var_v, 0, var_vw],
            [0, var_v, var_vw],
            [var_vw, var_vw, var_w]
        ])
    
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
        # Normalize heading
        s_new[2] = self.normalize_angle(s_new[2])

        # Propagate covariance using affine transform
        if self.motion_noise:
            #If using motion_model_noise, add motion-dependent process noise
            Q = self.noise(u[0], u[1])
            Sigma_new = A_k @ self.Sigma @ A_k.T + B_k @ Q @ B_k.T + self.covariance
        else:
            Sigma_new = A_k @ self.Sigma @ A_k.T + self.covariance
        return s_new, Sigma_new
    
    def kalman_filter(self, z, landmark_pos):
        """
        Update the robot's position using a Kalman Filter.

        Args:
            z (np.array): Measurement vector [range, bearing].
            landmark_pos (np.array): Landmark position [x, y].
        """
        ## -------- Using s_new ------- ##
        # Unpack the current state
        dx = landmark_pos[0] - self.s[0]
        dy = landmark_pos[1] - self.s[1]
        theta = self.s[2]

        eps = 1e-6
        p = max(dx**2 + dy**2, eps)  # Avoid division by zero

        # Expected measurement (eq. 20)
        z_hat = np.array([
            np.sqrt(p),
            np.arctan2(dy, dx) - theta
        ])
        z_hat[1] = self.normalize_angle(z_hat[1])

        # Measurement Jacobian (eq. 21)
        G = np.array([
            [-dx / np.sqrt(p), -dy / np.sqrt(p), 0],
            [ dy / p,          -dx / p,         -1]
        ])

        # Innovation covariance (eq. 22)
        S = G @ self.Sigma @ G.T + self.R

        # Kalman gain (eq. 23)
        K = self.Sigma @ G.T @ np.linalg.inv(S)

        # Innovation (difference between actual and expected measurement)
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])

        #Calculating NIS
        nis = innovation.T @ np.linalg.inv(S) @ innovation
        self.nis_values.append(nis)
        if len(self.nis_values) > self.nis_window_size:
            self.nis_values.pop(0)

        #Monitor NIS 
        self.monitor_consistency()

        ## ------- State (eq. 24) update Using s_new ------- ## 
        self.s += K @ innovation
        # Normalize heading
        self.s[2] = self.normalize_angle(self.s[2])

        # Covariance update (eq. 25)
        I = np.eye(3)
        # Covariance update (Joseph form )
        self.Sigma = (I - K @ G) @ self.Sigma @ (I - K @ G).T + K @ self.R @ K.T


    def monitor_consistency(self):
        if len(self.nis_values) < self.nis_window_size:
            return

        # Count values outside bounds
        too_high = sum(1 for nis in self.nis_values if nis > self.chi2_upper)
        too_low = sum(1 for nis in self.nis_values if nis < self.chi2_lower)

        high_percent = too_high / len(self.nis_values) * 100
        low_percent = too_low / len(self.nis_values) * 100

        # Log filter consistency status
        if high_percent > 15:  # More than 15% above upper bound
            self.get_logger().warn("Filter appears overconfident (NIS too large)")
        elif low_percent > 15:  # More than 15% below lower bound
            self.get_logger().warn("Filter appears underconfident (NIS too small)")

        
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
