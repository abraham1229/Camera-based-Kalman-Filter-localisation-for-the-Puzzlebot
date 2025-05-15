import rclpy
from rclpy.node import Node
from msgs_clase.msg import Goal   # type: ignore
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
import transforms3d
import signal
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')

        self.wait_for_ros_time()

        # Declare and get parameters
        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_yaw', 0.0)

        self.Posx = self.get_parameter('init_pose_x').value
        self.Posy = self.get_parameter('init_pose_y').value
        self.Postheta = self.get_parameter('init_pose_yaw').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.use_sim = self.get_parameter('use_sim_time').value

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1000)

        q = QoSProfile(depth=10)
        q.reliability = ReliabilityPolicy.RELIABLE

        # Timer
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Subscriptions

        self.ground_truth_sub = self.create_subscription(
            Odometry,
            'ground_truth',
            self.ground_truth_callback,
            q
        )

        self.subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.update_scan, 
            rclpy.qos.qos_profile_sensor_data,
        )
        
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

        self.hit_front = 0
        self.hit_left  = 0
        self.hit_right = 0

        # Initialize Lidar variables
        self.latest_scan = None
        self.obstacle_threshold = 0.3  # Obstacle detection distance threshold (meters)
        self.forward_angle_width = 30  # Degrees to check directly ahead (±15 degrees)
        self.left_width    = 60    # ±30° around +90°
        self.right_width   = 60    # ±30° around -90°

        # Control variables
        self.velL = 0.0
        self.velA = 0.0
        self.error_distancia = 0.0
        self.errorTheta = 0.0
        self.kpTheta = 0.8
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

        self.compute_errors()

        if self.check_goal_reached():
            return

        self.apply_control()
        self.limit_velocities()
        self.check_point_reached()
        self.publish_velocity_command()

    def update_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def lidar_scan(self):
        if self.latest_scan is None:
            self.get_logger().warn('No LaserScan data received yet.')
            return

        msg = self.latest_scan

        if msg is not None:
            ranges = np.array(msg.ranges)

            # Angle between each LiDAR measurement
            angle_increment = msg.angle_increment

            # Total number of points in one full 360-degree LiDAR scan
            num_points = len(ranges)

            def angle_to_index(angle_rad, angle_increment, num_points):
                # Raw index offset from forward
                raw_idx = int(round(angle_rad / angle_increment))
                # Wrap into [0, num_points)
                return raw_idx % num_points

            # Centers
            idx_forward = angle_to_index(0.0, angle_increment, num_points)
            idx_left    = angle_to_index( math.pi/2, angle_increment, num_points)
            idx_right   = angle_to_index(-math.pi/2, angle_increment, num_points)

            # Half‐widths in indices
            fw = int(round((math.radians(self.forward_angle_width) / 2) / angle_increment))
            lw = int(round((math.radians(self.left_width)    / 2) / angle_increment))
            rw = int(round((math.radians(self.right_width)   / 2) / angle_increment))

            # Build windows
            front_idxs = [(idx_forward + i) % num_points for i in range(-fw, fw+1)]
            left_idxs  = [(idx_left    + i) % num_points for i in range(-lw, lw+1)]
            right_idxs = [(idx_right   + i) % num_points for i in range(-rw, rw+1)]

            # Extract distances
            front_ranges = ranges[front_idxs]
            left_ranges  = ranges[left_idxs]
            right_ranges = ranges[right_idxs]

            # Check thresholds
            self.hit_front = np.any(front_ranges < self.obstacle_threshold)
            self.hit_left  = np.any(left_ranges  < self.obstacle_threshold)
            self.hit_right = np.any(right_ranges < self.obstacle_threshold)
                

    def callback_odometry(self, msg: Odometry):
        if msg is not None:
            self.Posx = msg.pose.pose.position.x
            self.Posy = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w

            _, _, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])  # Order: w, x, y, z
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

        ## Log calculated errors
        self.get_logger().info(f'Error Distance: {self.error_distancia}')
        self.get_logger().info(f'Objective Angle: {self.angulo_objetivo}')
        self.get_logger().info(f'Error Theta (after normalization): {self.errorTheta}')

    def apply_control(self):

        # Angular control
        self.velA = self.kpTheta * self.errorTheta
        # Linear control
        self.velL = self.kpLineal * self.error_distancia

    def check_goal_reached(self):
        if self.error_distancia < 0.05:
            self.get_logger().info('Objetivo alcanzado!')
            self.trayectoria_finalizda = True
            self.velL = 0.0
            self.velA = 0.0
            twist_msg = Twist()
            twist_msg.linear.x = self.velL
            twist_msg.angular.z = self.velA
            self.pub_cmd_vel.publish(twist_msg)
            return True
        return False

    def limit_velocities(self):
        if self.velA > 0.25:
            self.velA = 0.25

        if self.velL > 0.2:
            self.velL = 0.2

        if abs(self.errorTheta) > 0.05:
            self.velL = 0.0

    def check_point_reached(self):
        if self.errorTheta < 0.05 and self.errorTheta > -0.05 and self.error_distancia < 0.05:
            self.trayectoria_finalizda = True
    
    def publish_velocity_command(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.velL
        twist_msg.angular.z = self.velA
        self.pub_cmd_vel.publish(twist_msg)

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(twist_msg)
        self.get_logger().info('Stopping robot due to obstacle detection.')

    def wait_for_ros_time(self):
        self.get_logger().info('Waiting for ROS time to become active...')
        while rclpy.ok():
            now = self.get_clock().now()
            if now.nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'ROS time is active! Start time: {now.nanoseconds * 1e-9:.2f}s')

    def stop_handler(self, signum, frame):
        """Handles Ctrl+C (SIGINT)."""
        self.get_logger().info("Interrupt received! Stopping node...")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    m_t_p = Controller()
    signal.signal(signal.SIGINT, m_t_p.stop_handler)
    try:
        rclpy.spin(m_t_p)
    except SystemExit:
        m_t_p.get_logger().info('SystemExit triggered. Shutting down cleanly.')
    finally:
        m_t_p.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()